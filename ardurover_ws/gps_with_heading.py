#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64
from pymavlink import mavutil


class ArduPilotGpsNode(Node):
    def __init__(self):
        super().__init__('ardupilot_gps_node')
        connection_string = '/dev/ttyACM2'
        self.get_logger().info(f'Connecting to ArduPilot on {connection_string}...')

        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat received! Source Pixhawk connected.")

        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        # Separate publisher for heading in degrees (0 to 360)
        self.heading_pub = self.create_publisher(Float64, '/gps/heading', 10)

        # Track fix quality separately from position
        self._fix_type = -1  # GPS_FIX_TYPE_NO_GPS until we hear otherwise
        self._satellites_visible = 0

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        while True:
            msg = self.master.recv_match(
                type=['GLOBAL_POSITION_INT', 'GPS_RAW_INT'], blocking=False)
            if not msg:
                break

            if msg.get_type() == 'GPS_RAW_INT':
                self._fix_type = msg.fix_type
                self._satellites_visible = msg.satellites_visible
                continue

            # msg.get_type() == 'GLOBAL_POSITION_INT'
            if msg.lat == 0 and msg.lon == 0:
                continue

            if self._fix_type < 3:
                continue

            # Publish Fix
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'gps'

            gps_msg.status.status = NavSatStatus.STATUS_FIX
            gps_msg.status.service = NavSatStatus.SERVICE_GPS

            gps_msg.latitude = msg.lat / 1e7
            gps_msg.longitude = msg.lon / 1e7
            gps_msg.altitude = msg.alt / 1000.0  # mm -> m, AMSL

            # Define estimated variance in meters squared (e.g., standard deviation of 1.0m -> variance = 1.0^2 = 1.0)
            h_var = 1.0  # Horizontal variance (latitude/longitude)
            v_var = 2.0  # Vertical variance (altitude)

            # Populate diagonal of 3x3 covariance matrix [var_lat, 0, 0,  0, var_lon, 0,  0, 0, var_alt]
            gps_msg.position_covariance = [
                h_var, 0.0,   0.0,
                0.0,   h_var, 0.0,
                0.0,   0.0,   v_var
            ]

            # Set type to DIAGNOSTIC_APPROX (value = 1) or KNOWN (value = 2)
            gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN


            self.gps_pub.publish(gps_msg)

            # Publish Heading
            # MAVLink hdg is in centidegrees (cdeg). 65535 means invalid/unknown.
            if hasattr(msg, 'hdg') and msg.hdg != 65535:
                heading_msg = Float64()
                heading_msg.data = msg.hdg / 100.0  # centidegrees -> degrees
                self.heading_pub.publish(heading_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArduPilot GPS Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

