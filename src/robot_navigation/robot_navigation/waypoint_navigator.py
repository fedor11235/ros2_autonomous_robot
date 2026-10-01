#!/usr/bin/env python3
"""Autonomous waypoint navigator with reactive lidar obstacle avoidance.

The node drives the robot through an ordered list of (x, y) waypoints expressed
in a fixed frame (``odom`` by default) while avoiding obstacles seen by the 2D
lidar. The controller is a hybrid of two behaviours:

  * go-to-goal       -- proportional heading control toward the active waypoint;
  * reactive avoid   -- when the front sector of the lidar sees something inside
                        ``avoid_distance`` the robot steers toward the freer side
                        (gap following) and slows/stops forward motion.

This keeps the system fully self-contained and reproducible: it needs only
/odom and /scan and emits /cmd_vel, with no map or external planner required.
A Nav2-based alternative is documented in docs/architecture.md.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from robot_navigation.geometry import clamp, normalize_angle, yaw_from_quaternion


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("waypoint_navigator")

        # ---------------- Parameters ----------------
        # Waypoints come in as a flat [x0, y0, x1, y1, ...] array of doubles.
        self.declare_parameter("waypoints", [4.0, 0.0, 4.0, 4.0])
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("control_rate", 20.0)

        self.declare_parameter("goal_tolerance", 0.25)
        self.declare_parameter("max_linear", 0.45)
        self.declare_parameter("max_angular", 1.4)
        self.declare_parameter("kp_angular", 2.0)
        # Heading error (rad) above which we rotate in place before advancing.
        self.declare_parameter("heading_align", 0.6)

        # Obstacle-avoidance geometry.
        self.declare_parameter("front_half_angle", 0.52)   # ~30 deg each side
        self.declare_parameter("side_half_angle", 1.22)    # ~70 deg sector used for clearance
        self.declare_parameter("slow_distance", 1.2)       # begin slowing
        self.declare_parameter("avoid_distance", 0.6)      # begin active steering
        self.declare_parameter("stop_distance", 0.35)      # pure rotation, no forward

        flat = list(self.get_parameter("waypoints").value)
        if len(flat) % 2 != 0:
            self.get_logger().warn("Odd number of waypoint coords; dropping last value.")
            flat = flat[:-1]
        self.waypoints = [(flat[i], flat[i + 1]) for i in range(0, len(flat), 2)]
        self.frame_id = self.get_parameter("frame_id").value

        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.max_linear = self.get_parameter("max_linear").value
        self.max_angular = self.get_parameter("max_angular").value
        self.kp_angular = self.get_parameter("kp_angular").value
        self.heading_align = self.get_parameter("heading_align").value
        self.front_half_angle = self.get_parameter("front_half_angle").value
        self.side_half_angle = self.get_parameter("side_half_angle").value
        self.slow_distance = self.get_parameter("slow_distance").value
        self.avoid_distance = self.get_parameter("avoid_distance").value
        self.stop_distance = self.get_parameter("stop_distance").value

        # ---------------- State ----------------
        self.pose = None            # (x, y, yaw)
        self.scan = None            # latest LaserScan
        self.wp_index = 0
        self.finished = False
        self._avoid_latch = 0.0     # committed avoidance turn dir (anti-chatter)

        # ---------------- I/O ----------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.path_pub = self.create_publisher(Path, "planned_path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "waypoint_markers", 10)

        self.create_subscription(Odometry, "odom", self.odom_cb, 10)
        self.create_subscription(LaserScan, "scan", self.scan_cb, sensor_qos)

        rate = self.get_parameter("control_rate").value
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        # Publish static viz a couple of times so RViz latches onto it.
        self.create_timer(1.0, self.publish_visuals)

        self.get_logger().info(
            f"WaypointNavigator up: {len(self.waypoints)} waypoints in '{self.frame_id}'.")

    # ---------------- Callbacks ----------------
    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    # ---------------- Lidar helpers ----------------
    def sector_min(self, center, half_width):
        """Minimum valid range within [center-half_width, center+half_width].

        ``center`` is relative to the lidar's forward axis (0 = straight ahead).
        Returns ``inf`` when the sector has no valid return.
        """
        scan = self.scan
        if scan is None:
            return math.inf
        best = math.inf
        n = len(scan.ranges)
        for i in range(n):
            angle = scan.angle_min + i * scan.angle_increment
            d = normalize_angle(angle - center)
            if abs(d) > half_width:
                continue
            r = scan.ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            best = min(best, r)
        return best

    # ---------------- Main control ----------------
    def control_loop(self):
        if self.finished:
            return
        if self.pose is None:
            return  # wait for first odom

        x, y, yaw = self.pose
        tx, ty = self.waypoints[self.wp_index]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)

        # Reached current waypoint?
        if dist < self.goal_tolerance:
            if self.wp_index + 1 < len(self.waypoints):
                self.wp_index += 1
                self.get_logger().info(
                    f"Reached waypoint {self.wp_index}/{len(self.waypoints)}; "
                    f"heading to {self.waypoints[self.wp_index]}.")
                return
            # Final waypoint reached -> stop.
            self.finished = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Final waypoint reached. Goal complete. Stopping.")
            return

        bearing = math.atan2(dy, dx)
        heading_err = normalize_angle(bearing - yaw)

        # Clearance readings.
        front = self.sector_min(0.0, self.front_half_angle)
        left = self.sector_min(self.side_half_angle, self.front_half_angle)
        right = self.sector_min(-self.side_half_angle, self.front_half_angle)

        cmd = Twist()

        # Release the avoidance latch as soon as the path ahead is clear.
        if front >= self.avoid_distance:
            self._avoid_latch = 0.0

        # The robot steers well only while moving (a stationary diff-drive just
        # slips its wheels instead of turning), so EVERY branch keeps a forward
        # velocity and turns as an arc -- the controller never pivots in place.
        if front < self.avoid_distance:
            # ---- Reactive obstacle avoidance ----
            # Latch the turn direction (toward the freer side) so it doesn't
            # chatter when left/right clearances are similar, and arc away.
            if self._avoid_latch == 0.0:
                self._avoid_latch = 1.0 if left >= right else -1.0
            cmd.angular.z = self._avoid_latch * self.max_angular
            cmd.linear.x = 0.12
        else:
            # ---- Go-to-goal (pure-pursuit style arcing) ----
            clear_scale = 1.0
            if front < self.slow_distance:
                clear_scale = clamp(
                    (front - self.avoid_distance) /
                    max(1e-3, (self.slow_distance - self.avoid_distance)),
                    0.3, 1.0)
            # Slow (but never stop) when badly misaligned so the turn is tighter.
            heading_scale = clamp(1.0 - abs(heading_err) / math.pi, 0.35, 1.0)
            speed = self.max_linear * heading_scale * clear_scale
            speed = min(speed, 0.6 * dist + 0.1)          # ease in near the waypoint
            cmd.linear.x = clamp(max(speed, 0.12), 0.0, self.max_linear)
            cmd.angular.z = clamp(self.kp_angular * heading_err,
                                  -self.max_angular, self.max_angular)

        self.cmd_pub.publish(cmd)

    # ---------------- Visualisation ----------------
    def publish_visuals(self):
        now = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = now
        for (wx, wy) in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.header.stamp = now
            ps.pose.position.x = float(wx)
            ps.pose.position.y = float(wy)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        markers = MarkerArray()
        for i, (wx, wy) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(wx), y=float(wy), z=0.15)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            last = (i == len(self.waypoints) - 1)
            m.color = ColorRGBA(r=0.9 if last else 0.1,
                                g=0.1 if last else 0.8,
                                b=0.1, a=0.9)
            markers.markers.append(m)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best-effort stop on shutdown.
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
