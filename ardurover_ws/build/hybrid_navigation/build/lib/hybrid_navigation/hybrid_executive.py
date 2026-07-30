#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math

class HybridExecutive(Node):
    def __init__(self):
        super().__init__('hybrid_executive')
        
        self.navigator = BasicNavigator()
        self.mode_client = self.create_client(SetBool, '/switch_ap_mode')
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
        self.avoiding_obstacle = False
        self.critical_distance = 3.0  # Meters
        self.avoidance_distance = 5.0 # How far ahead to set the Nav2 goal

        # Wait for the HITL bridge service to be available
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /switch_ap_mode service...')

    def scan_cb(self, msg: LaserScan):
        if self.avoiding_obstacle:
            return 

        # Check center 30 degrees of LiDAR
        center_idx = len(msg.ranges) // 2
        window = 15
        front_ranges = msg.ranges[center_idx - window : center_idx + window]
        min_dist = min([r for r in front_ranges if not math.isinf(r) and not math.isnan(r)], default=float('inf'))

        if min_dist < self.critical_distance:
            self.get_logger().warn(f"Obstacle detected at {min_dist:.2f}m. Triggering Nav2.")
            self.trigger_avoidance()

    def trigger_avoidance(self):
        self.avoiding_obstacle = True
        
        # 1. Put ArduPilot into HOLD
        req = SetBool.Request()
        req.data = True
        self.mode_client.call_async(req)

        # 2. Set goal 5 meters straight ahead in the robot's local frame
        # Nav2 automatically transforms 'base_link' to the global map/odom frame internally
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'base_link'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(self.avoidance_distance)
        goal_pose.pose.orientation.w = 1.0

        # 3. Dispatch to Nav2
        self.navigator.goToPose(goal_pose)

        # 4. Block and monitor (in a real system, use a state machine or timer to avoid blocking callbacks)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        # 5. Restore ArduPilot to AUTO
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Avoidance successful. Returning control to ArduPilot.")
        else:
            self.get_logger().error("Nav2 failed to reach avoidance target. Returning to ArduPilot anyway.")

        req.data = False
        self.mode_client.call_async(req)
        self.avoiding_obstacle = False

def main():
    rclpy.init()
    node = HybridExecutive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
