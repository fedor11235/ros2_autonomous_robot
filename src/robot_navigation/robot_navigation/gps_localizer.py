#!/usr/bin/env python3
"""Convert GPS fixes (sensor_msgs/NavSatFix) into a local ENU pose.

The first valid fix becomes the datum (local origin). Subsequent fixes are
projected to local East-North-Up metres with an equirectangular approximation,
which is accurate to well under a centimetre over the ~10 m course here. The
result is published as:

  * /gps/odom  (nav_msgs/Odometry) -- x=East, y=North in the ``gps`` frame;
  * a log line with the horizontal error vs. the configured goal, so GPS
    progress is observable even without RViz.

This shows genuine use of the GPS sensor independent of wheel odometry; a
production system would instead fuse both in robot_localization (see docs).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

EARTH_RADIUS = 6378137.0  # WGS-84 semi-major axis, metres


class GpsLocalizer(Node):
    def __init__(self):
        super().__init__("gps_localizer")

        self.declare_parameter("frame_id", "gps")
        self.declare_parameter("child_frame_id", "base_footprint")
        # Optional goal in local ENU metres, for a human-readable progress log.
        self.declare_parameter("goal_east", 4.0)
        self.declare_parameter("goal_north", 4.0)
        self.declare_parameter("log_period", 2.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.goal_e = self.get_parameter("goal_east").value
        self.goal_n = self.get_parameter("goal_north").value

        self.datum = None     # (lat0, lon0, alt0) in radians/metres
        self.last_enu = None   # (e, n, u)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.odom_pub = self.create_publisher(Odometry, "gps/odom", 10)
        self.create_subscription(NavSatFix, "gps/fix", self.fix_cb, sensor_qos)

        period = self.get_parameter("log_period").value
        self.create_timer(period, self.log_progress)

        self.get_logger().info("GpsLocalizer up: waiting for first fix to set datum.")

    def fix_cb(self, msg: NavSatFix):
        # STATUS_NO_FIX == -1; ignore invalid fixes.
        if msg.status.status < 0:
            return
        lat = math.radians(msg.latitude)
        lon = math.radians(msg.longitude)
        alt = msg.altitude

        if self.datum is None:
            self.datum = (lat, lon, alt)
            self.get_logger().info(
                f"GPS datum set: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, "
                f"alt={alt:.1f}")
            return

        lat0, lon0, alt0 = self.datum
        east = (lon - lon0) * math.cos(lat0) * EARTH_RADIUS
        north = (lat - lat0) * EARTH_RADIUS
        up = alt - alt0
        self.last_enu = (east, north, up)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = east
        odom.pose.pose.position.y = north
        odom.pose.pose.position.z = up
        odom.pose.pose.orientation.w = 1.0
        # Rough position covariance from the sensor's reported stddev.
        var = 0.8 * 0.8
        odom.pose.covariance[0] = var
        odom.pose.covariance[7] = var
        odom.pose.covariance[14] = 1.5 * 1.5
        self.odom_pub.publish(odom)

    def log_progress(self):
        if self.last_enu is None:
            return
        e, n, _ = self.last_enu
        err = math.hypot(self.goal_e - e, self.goal_n - n)
        self.get_logger().info(
            f"GPS local ENU: E={e:+.2f} N={n:+.2f} m | distance to goal: {err:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = GpsLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
