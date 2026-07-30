#!/usr/bin/env python3

import rclpy
import threading
import time
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math

import tf2_ros
import tf2_geometry_msgs  # noqa: F401


class HybridExecutive(Node):
    def __init__(self):
        super().__init__('hybrid_executive')

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.mode_client = self.create_client(SetBool, '/switch_ap_mode')
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # BUG-L1 FIX: Publisher for the twist_mux lock topic. Publishing True
        # suppresses the lower-priority /cmd_vel_ardupilot source while Nav2
        # avoidance is active, preventing ArduPilot and Nav2 from fighting.
        self._nav2_lock_pub = self.create_publisher(Bool, '/nav2_lock', 10)

        self.critical_distance = 1.0   # metres — obstacle trigger distance
        self.avoidance_distance = 2.0  # metres — how far ahead to navigate

        # TF2 for base_link → odom transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # BUG-C1 FIX: Protect the avoiding_obstacle flag with a Lock so the
        # check-and-set in trigger_avoidance() is atomic. Without this, scan_cb
        # could spawn multiple avoidance threads if it fires between the flag
        # read and the flag write.
        self._avoidance_lock = threading.Lock()
        self._avoiding_obstacle = False

        # /switch_ap_mode is optional — only present when HITL bridge is running.
        if self.mode_client.service_is_ready():
            self.get_logger().info(
                '/switch_ap_mode available — ArduPilot mode switching enabled.'
            )
        else:
            self.get_logger().warn(
                '/switch_ap_mode not available. '
                'Running in Nav2-only mode (no ArduPilot mode switching).'
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _set_nav2_lock(self, locked: bool):
        """Publish True to twist_mux lock topic to suppress ArduPilot source."""
        msg = Bool()
        msg.data = locked
        self._nav2_lock_pub.publish(msg)

    def _call_mode_switch(self, enable: bool):
        """Switch ArduPilot mode only if the HITL bridge service is running."""
        if not self.mode_client.service_is_ready():
            self.get_logger().warn('Cannot switch AP mode: /switch_ap_mode service is not available!')
            return

        mode_str = 'HOLD' if enable else 'AUTO'
        self.get_logger().info(f'Requesting ArduPilot mode switch to {mode_str}...')

        req = SetBool.Request()
        req.data = enable
        future = self.mode_client.call_async(req)

        # BUG-M1 FIX: Increased timeout from 2 s → 5 s. The future resolves
        # only when the main thread's executor processes the response. 2 s was
        # too short if the main thread was briefly busy with another callback.
        start_time = time.time()
        while rclpy.ok() and not future.done() and (time.time() - start_time) < 5.0:
            time.sleep(0.05)

        if future.done():
            res = future.result()
            if res and res.success:
                self.get_logger().info(f'Successfully switched to {mode_str} mode.')
            else:
                self.get_logger().error(f'Failed to switch to {mode_str} mode!')
        else:
            self.get_logger().error('Timeout waiting for /switch_ap_mode response!')

    def _cleanup_avoidance(self):
        """Release twist_mux lock, restore ArduPilot AUTO mode, and clear flag.

        Extracted as a helper so every exit path in _avoidance_thread (early
        return on TF failure, goal rejection, or normal completion) goes through
        the same teardown, guaranteeing the lock and flag are always released.
        """
        self._set_nav2_lock(False)
        self._call_mode_switch(False)
        with self._avoidance_lock:
            self._avoiding_obstacle = False

    # ------------------------------------------------------------------
    # LiDAR callback
    # ------------------------------------------------------------------

    def scan_cb(self, msg: LaserScan):
        # BUG-C1 FIX: Read the flag under the lock so this check is consistent
        # with the atomic check-and-set in trigger_avoidance().
        with self._avoidance_lock:
            if self._avoiding_obstacle:
                return

        center_idx = len(msg.ranges) // 2
        window = 15
        front_ranges = msg.ranges[center_idx - window: center_idx + window]
        min_dist = min(
            [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)],
            default=float('inf'),
        )

        if min_dist < self.critical_distance:
            self.get_logger().warn(
                f'Obstacle at {min_dist:.2f}m — triggering Nav2 avoidance.'
            )
            self.trigger_avoidance()

    # ------------------------------------------------------------------
    # Avoidance
    # ------------------------------------------------------------------

    def trigger_avoidance(self):
        # BUG-C1 FIX: Check-and-set is now atomic under the lock. Even if
        # scan_cb fires multiple times before the thread starts, only one
        # avoidance thread will ever be spawned.
        with self._avoidance_lock:
            if self._avoiding_obstacle:
                return
            self._avoiding_obstacle = True
        threading.Thread(target=self._avoidance_thread, daemon=True).start()

    def _avoidance_thread(self):
        """Runs in a background thread — must NOT call rclpy.spin*."""

        # BUG-C2 FIX: Removed the _nav2_ready threading.Event that was set
        # immediately in main() (before Nav2 had started). It gave a false
        # sense of readiness. wait_for_server() below is the correct and safe
        # way to wait for the Nav2 action server from a background thread.

        # (Optional) Suppress ArduPilot output and put it in HOLD
        self._set_nav2_lock(True)
        self._call_mode_switch(True)

        # Build goal 2 m ahead in base_link, transform to odom
        goal_in_base = PoseStamped()
        goal_in_base.header.frame_id = 'base_link'
        goal_in_base.header.stamp = self.get_clock().now().to_msg()
        goal_in_base.pose.position.x = float(self.avoidance_distance)
        goal_in_base.pose.orientation.w = 1.0

        try:
            goal_pose = self.tf_buffer.transform(
                goal_in_base, 'odom',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            goal_pose.header.frame_id = 'odom'
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'TF base_link→odom failed: {e}. Aborting.')
            self._cleanup_avoidance()
            return

        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self.nav_to_pose_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info('Sending avoidance goal to Nav2...')

        # Send goal async and block-poll (main thread is spinning, so the future
        # will be resolved by it without any rclpy calls from this thread).
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the avoidance goal or connection failed! Aborting.')
            self._cleanup_avoidance()
            return

        self.get_logger().info('Goal accepted, executing...')

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)

        action_result = result_future.result()
        result_status = action_result.status if action_result else -1

        if result_status == 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info('Avoidance succeeded. Returning to ArduPilot.')
        else:
            self.get_logger().error(
                f'Nav2 avoidance failed (status {result_status}). Returning anyway.'
            )

        self._cleanup_avoidance()


# ----------------------------------------------------------------------
# Entry point
# ----------------------------------------------------------------------

def main():
    rclpy.init()
    node = HybridExecutive()

    # BUG-C2 FIX: Removed the immediately-set _nav2_ready event. Nav2
    # readiness is handled correctly by wait_for_server() inside
    # _avoidance_thread(), which is the only safe way to do it.
    node.get_logger().info('HybridExecutive started. Waiting for obstacles...')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
