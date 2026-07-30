#!/usr/bin/env python3
"""
hitl_1.py

Hardware-in-the-loop bridge between:
  - ArduPilot's `ardurover` binary running natively on the Jetson
    (launched with --model JSON:<this-host-ip>:<listen-port>)
  - your existing ROS2 sensor/actuator stack (BNO055 IMU, GPS, wheel
    odometry, motor driver via /cmd_vel)

ArduPilot's EKF/nav/control code runs unmodified inside the `ardurover`
binary. This bridge stands in for the "physics backend" that SITL
normally simulates - except instead of fake physics, it's real sensors
and a real motor driver.
"""

import argparse
import json
import math
import socket
import struct
import threading
import time

from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

MAGIC_16CH = 18458
MAGIC_32CH = 29569
HEADER_FMT = "<HHI"  # magic, frame_rate, frame_count
HEADER_SIZE = struct.calcsize(HEADER_FMT)
PWM16_FMT = "<16H"
PWM32_FMT = "<32H"

EARTH_RADIUS_M = 6378137.0  # flat-earth approx, fine at rover-scale distances


def quat_ros_to_ardupilot(x, y, z, w):
    """ROS orientation is (x,y,z,w) in a FLU body frame. ArduPilot wants
    (q1,q2,q3,q4) = (w,x,y,z) in its FRD body frame. FLU -> FRD is a
    180 deg rotation about X, which just negates y and z."""
    return (w, x, -y, -z)


def latlon_to_ned(lat, lon, alt, origin_lat, origin_lon, origin_alt):
    dlat = math.radians(lat - origin_lat)
    dlon = math.radians(lon - origin_lon)
    north = dlat * EARTH_RADIUS_M
    east = dlon * EARTH_RADIUS_M * math.cos(math.radians(origin_lat))
    down = -(alt - origin_alt)
    return north, east, down


class ArduRoverHITLBridge(Node):
    def __init__(self, args):
        super().__init__("ardurover_hitl_bridge")
        self.args = args

        # ---- shared state, guarded by one lock ----
        self._state_lock = threading.Lock()
        self._imu_gyro = [0.0, 0.0, 0.0]
        self._imu_accel = [0.0, 0.0, 0.0]
        self._quat = None                # (w,x,y,z) ArduPilot convention
        self._gps_origin = None          # (lat, lon, alt), set on first fix
        self._fc_origin_synced = False   # whether SET_GPS_GLOBAL_ORIGIN has reached ardurover
        self._gps_heading = None         # heading in degrees (0.0 to 360.0)
        self._ned_pos = [0.0, 0.0, 0.0]
        self._ned_vel = [0.0, 0.0, 0.0]
        self._last_pwm = [1500] * 16
        self._smoothed_linear = 0.0
        self._smoothed_angular = 0.0
        self._last_gps_send_time = 0.0
        self._last_frame_count = -1
        self._filtered_left_pwm = 1500.0
        self._filtered_right_pwm = 1500.0

        self.motor_pub_rate = 20.0  # Hz (Match what your motor driver prefers, usually 20-50 Hz)
        self.create_timer(1.0 / self.motor_pub_rate, self._motor_control_timer_cb)
        self._last_cmd_vel_time = time.monotonic()

        # ---- UDP link to the ardurover binary ----
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((args.listen_ip, args.listen_port))
        self._sock.settimeout(1.0)
        self._ap_addr = None             # learned from the first packet in
        self._send_lock = threading.Lock()
        self._stop = False

        # ---- ROS2 I/O ----
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Imu, args.imu_topic, self._on_imu, qos)
        self.create_subscription(Imu, args.accel_topic, self._on_imu_raw, qos)
        self.create_subscription(NavSatFix, args.gps_topic, self._on_gps, qos)
        self.create_subscription(Float64, args.heading_topic, self._on_heading, qos)
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, qos)

        self.cmd_vel_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)

        self._udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
        self._udp_thread.start()

        # ---- armed-state watchdog ----
        self._armed = False
        self._mav_conn = None
        self._mav_conn_lock = threading.Lock()
        self._mav_thread = threading.Thread(target=self._mavlink_watch_loop, daemon=True)
        self._mav_thread.start()

        self.get_logger().info(
            f"HITL bridge up. Listening on {args.listen_ip}:{args.listen_port}. "
            f"Launch ardurover with --model JSON:127.0.0.1:{args.listen_port}"
        )

    # ------------------------------------------------------------------
    # ROS2 sensor callbacks
    # ------------------------------------------------------------------

    def _on_imu(self, msg: Imu):
        with self._state_lock:
            gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            # FLU -> FRD body frame: negate y and z
            self._imu_gyro = [gx, -gy, -gz]

            q = msg.orientation
            if not (q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0):
                self._quat = quat_ros_to_ardupilot(q.x, q.y, q.z, q.w)

    def _on_imu_raw(self, msg: Imu):
        with self._state_lock:
            ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
            # FLU -> FRD body frame: negate y and z
            self._imu_accel = [ax, -ay, -az]

    def _on_heading(self, msg: Float64):
        with self._state_lock:
            heading = msg.data % 360.0
            if heading < 0:
                heading += 360.0
            self._gps_heading = heading

    def _on_gps(self, msg: NavSatFix):
        if msg.status.status < 0:
            return

        now = time.monotonic()

        with self._state_lock:
            if self._gps_origin is None:
                self._gps_origin = (msg.latitude, msg.longitude, msg.altitude)
                self.get_logger().info(f"GPS origin latched at {self._gps_origin}")

            n, e, d = latlon_to_ned(
                msg.latitude, msg.longitude, msg.altitude, *self._gps_origin
            )
            self._ned_pos = [n, e, d]
            vn, ve, vd = self._ned_vel
            need_origin_sync = not self._fc_origin_synced
            current_heading = self._gps_heading

        # Try to read real GPS quality values if your driver publishes them.
        # Standard sensor_msgs/NavSatFix does not define num_sats/hdop/vdop,
        # so we fall back to safe defaults when those fields are unavailable.
        num_sats = getattr(msg, "num_sats", None)
        if num_sats is None:
            num_sats = getattr(msg, "satellites", None)
        if num_sats is None:
            num_sats = getattr(msg, "sat_count", None)
        if num_sats is None:
            num_sats = 12

        hdop = getattr(msg, "hdop", None)
        vdop = getattr(msg, "vdop", None)
        if hdop is None or vdop is None:
            cov = getattr(msg, "position_covariance", None)
            if cov and len(cov) >= 9:
                horiz = math.sqrt(max(cov[0], 0.0) + max(cov[4], 0.0))
                vert = math.sqrt(max(cov[8], 0.0))
                hdop = max(0.5, min(10.0, horiz / 3.0)) if horiz > 0 else 1.0
                vdop = max(0.5, min(10.0, vert / 3.0)) if vert > 0 else 1.0
            else:
                hdop = 1.0
                vdop = 1.0

        # rate-limit MAVLink GPS_INPUT to 10 Hz
        if (now - self._last_gps_send_time) >= 0.1:
            self._last_gps_send_time = now
            self._send_gps_input(
                msg.latitude,
                msg.longitude,
                msg.altitude,
                vn,
                ve,
                vd,
                num_sats=num_sats,
                heading_deg=current_heading,
                hdop=hdop,
                vdop=vdop,
                h_acc=1.0,
                v_acc=1.5,
            )

        if need_origin_sync:
            sent = self._send_gps_global_origin(
                msg.latitude, msg.longitude, msg.altitude
            )
            if sent:
                with self._state_lock:
                    self._fc_origin_synced = True

    def _on_odom(self, msg: Odometry):
        with self._state_lock:
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            if self._quat is not None:
                w, x, y, z = self._quat
                yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            else:
                yaw = 0.0
            # rotate body-frame velocity into NED using current heading
            self._ned_vel = [
                vx * math.cos(yaw) - vy * math.sin(yaw),
                vx * math.sin(yaw) + vy * math.cos(yaw),
                0.0,
            ]

    # ------------------------------------------------------------------
    # Armed-state watchdog
    # ------------------------------------------------------------------

    def _mavlink_watch_loop(self):
        while not self._stop:
            try:
                conn = mavutil.mavlink_connection(self.args.mavlink_url)
                conn.wait_heartbeat(timeout=5)
                with self._mav_conn_lock:
                    self._mav_conn = conn
                self.get_logger().info(f"Armed-state watchdog connected via {self.args.mavlink_url}")
                while not self._stop:
                    msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
                    if msg is None:
                        continue
                    
                    # Ignore heartbeats from non-autopilot components (like GCS or MAVProxy)
                    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                        continue

                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    with self._state_lock:
                        if armed != self._armed:
                            self.get_logger().info(f"Vehicle armed state -> {armed}")
                        self._armed = armed
            except (OSError, ConnectionError) as e:
                with self._state_lock:
                    self._armed = False
                with self._mav_conn_lock:
                    self._mav_conn = None
                self.get_logger().warn(f"Armed-state watchdog link lost ({e}), retrying...")
                time.sleep(2.0)

    def _send_gps_input(self, lat, lon, alt, vn, ve, vd, num_sats,
                         heading_deg=None, hdop=1.0, vdop=1.0,
                         h_acc=1.0, v_acc=1.5):
        """Inject real GPS coordinates and heading into ArduPilot via GPS_INPUT."""
        with self._mav_conn_lock:
            conn = self._mav_conn
        if conn is None:
            return

        if heading_deg is not None:
            yaw_cdeg = int(round(heading_deg * 100.0)) % 36000
        else:
            yaw_cdeg = 0

        time_us = int(time.time() * 1e6)

        try:
            conn.mav.gps_input_send(
                time_us,                 # time_usec
                0,                       # gps_id
                0,                       # ignore_flags - use everything
                0, 0,                    # time_week_ms, time_week
                3,                       # fix_type: 3 = 3D fix
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt),
                float(hdop),
                float(vdop),
                float(vn), float(ve), float(vd),
                0.2,                     # speed_accuracy
                float(h_acc),
                float(v_acc),
                max(num_sats, 8),
                yaw_cdeg,
            )
        except OSError as e:
            self.get_logger().warn(f"GPS_INPUT send failed: {e}")

    def _send_gps_global_origin(self, lat, lon, alt):
        with self._mav_conn_lock:
            conn = self._mav_conn
        if conn is None:
            self.get_logger().warn("Can't set GPS global origin yet - no MAVLink connection "
                                   "(will keep retrying on subsequent fixes)")
            return False
        try:
            target_sys = getattr(conn, "target_system", 1) or 1
            conn.mav.set_gps_global_origin_send(
                target_sys,
                int(lat * 1e7),
                int(lon * 1e7),
                int(alt * 1000),  # mm
                int(time.time() * 1e6),
            )
            self.get_logger().info(f"Sent SET_GPS_GLOBAL_ORIGIN: {lat}, {lon}, {alt}m")
            return True
        except OSError as e:
            self.get_logger().warn(f"SET_GPS_GLOBAL_ORIGIN send failed: {e}")
            return False

    # ------------------------------------------------------------------
    # UDP / JSON HITL loop
    # ------------------------------------------------------------------

    def _udp_loop(self):
        while not self._stop:
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break

            self._ap_addr = addr
            parsed = self._parse_pwm_packet(data)
            if parsed is None:
                continue
            _frame_count, pwm = parsed

            with self._state_lock:
                self._last_pwm = pwm

            self._send_sensor_frame()

    def _parse_pwm_packet(self, data: bytes):
        if len(data) < HEADER_SIZE:
            return None
        magic, _frame_rate, frame_count = struct.unpack_from(HEADER_FMT, data, 0)
        if magic == MAGIC_16CH:
            fmt = PWM16_FMT
        elif magic == MAGIC_32CH:
            fmt = PWM32_FMT
        else:
            self.get_logger().warn(f"Unrecognised magic {magic}, dropping packet")
            return None
        if len(data) < HEADER_SIZE + struct.calcsize(fmt):
            return None
        pwm = struct.unpack_from(fmt, data, HEADER_SIZE)
        if frame_count != self._last_frame_count + 1:
            self.get_logger().warn(
                f"PWM frame gap: expected {self._last_frame_count + 1}, got {frame_count}"
            )
        self._last_frame_count = frame_count
        return frame_count, list(pwm)

    def _send_sensor_frame(self):
        if self._ap_addr is None:
            return
        with self._state_lock:
            gyro, accel, quat = self._imu_gyro, self._imu_accel, self._quat
            pos, vel = self._ned_pos, self._ned_vel

        frame = {
            "timestamp": time.monotonic(),
            "imu": {"gyro": gyro, "accel_body": accel},
            "position": pos,
            "velocity": vel,
        }
        if quat is not None:
            frame["quaternion"] = list(quat)
        else:
            frame["attitude"] = [0.0, 0.0, 0.0]

        payload = (json.dumps(frame) + "\n").encode("ascii")
        with self._send_lock:
            try:
                self._sock.sendto(payload, self._ap_addr)
            except OSError as e:
                self.get_logger().warn(f"UDP send failed: {e}")

    # ------------------------------------------------------------------
    # PWM -> motor command
    # ------------------------------------------------------------------

    @staticmethod
    def _pwm_to_normalized(pwm_us, deadzone=100, span=700.0, center=1500):
        if pwm_us < 800 or pwm_us > 2200:
            return 0.0
        centered = pwm_us - center
        if abs(centered) < deadzone:
            return 0.0
        return max(-1.0, min(1.0, centered / span))

    def _smooth_pwm_value(self, raw_pwm, filtered_pwm, alpha=0.25):
        return filtered_pwm + alpha * (raw_pwm - filtered_pwm)

    def _motor_control_timer_cb(self):
        now = time.monotonic()
        dt = now - self._last_cmd_vel_time
        self._last_cmd_vel_time = now

        with self._state_lock:
            pwm = list(self._last_pwm)
            armed = self._armed

        if not armed:
            self.cmd_vel_pub.publish(Twist())
            self._smoothed_linear = 0.0
            self._smoothed_angular = 0.0
            return

        raw_left_pwm = pwm[self.args.left_channel - 1]
        raw_right_pwm = pwm[self.args.right_channel - 1]
        left_pwm = self._smooth_pwm_value(raw_left_pwm, self._filtered_left_pwm)
        right_pwm = self._smooth_pwm_value(raw_right_pwm, self._filtered_right_pwm)
        self._filtered_left_pwm = left_pwm
        self._filtered_right_pwm = right_pwm

        left = self._pwm_to_normalized(left_pwm, deadzone=100, span=700.0)
        right = self._pwm_to_normalized(right_pwm, deadzone=100, span=700.0)

        target_linear = (left + right) / 2.0 * self.args.max_linear_speed
        target_angular = (right - left) / self.args.wheel_track * self.args.max_linear_speed

        # Time-consistent Low-Pass Filter (Cutoff frequency ~3-5 Hz)
        # rc = 1 / (2 * pi * cutoff_freq)
        cutoff_freq = 4.0  # Hz
        rc = 1.0 / (2.0 * math.pi * cutoff_freq)
        alpha = dt / (rc + dt) if (rc + dt) > 0 else 1.0

        if self.args.smoothing_alpha == 1.0:
            alpha = 1.0  # Respect user flag if smoothing disabled

        self._smoothed_linear += alpha * (target_linear - self._smoothed_linear)
        self._smoothed_angular += alpha * (target_angular - self._smoothed_angular)

        twist = Twist()
        twist.linear.x = self._smoothed_linear
        twist.angular.z = self._smoothed_angular
        self.cmd_vel_pub.publish(twist)

    def destroy(self):
        self._stop = True
        self._sock.close()


def parse_args():
    p = argparse.ArgumentParser(description="ArduRover JSON HITL bridge for Jetson")
    p.add_argument("--listen-ip", default="0.0.0.0")
    p.add_argument("--listen-port", type=int, default=9002)
    p.add_argument("--mavlink-url", default="tcp:127.0.0.1:5760",
                    help="MAVLink connection used purely to track real arm state")
    p.add_argument("--imu-topic", default="/data",
                    help="fused sensor_msgs/Imu topic - used for orientation and gyro only")
    p.add_argument("--accel-topic", default="/raw",
                    help="raw sensor_msgs/Imu topic - used for acceleration only "
                         "(must include gravity, i.e. ~9.8 m/s^2 at rest, unlike /data)")
    p.add_argument("--gps-topic", default="/gps/fix",
                    help="sensor_msgs/NavSatFix topic")
    p.add_argument("--heading-topic", default="/gps/heading",
                    help="std_msgs/Float64 topic carrying GPS heading in degrees")
    p.add_argument("--odom-topic", default="/r1a004/wheel_odom")
    p.add_argument("--cmd-vel-topic", default="/cmd_vel")
    p.add_argument("--left-channel", type=int, default=1,
                    help="1-based SERVOn output channel mapped to the left motor")
    p.add_argument("--right-channel", type=int, default=3,
                    help="1-based SERVOn output channel mapped to the right motor")
    p.add_argument("--wheel-track", type=float, default=0.27,
                    help="metres between left/right wheel contact points")
    p.add_argument("--max-linear-speed", type=float, default=1.32,
                    help="m/s at full PWM deflection")
    p.add_argument("--smoothing-alpha", type=float, default=0.3,
                    help="0-1: exponential smoothing on cmd_vel output. Lower = smoother "
                         "but more lag, higher = snappier but closer to raw jumpy PWM. "
                         "1.0 disables smoothing entirely.")
    return p.parse_args()


def main():
    rclpy.init()
    node = ArduRoverHITLBridge(parse_args())
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
