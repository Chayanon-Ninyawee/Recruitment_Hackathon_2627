#!/usr/bin/env python3
"""YOUR DRIVER GOES HERE.

This is the node the judges run. Keep `driver` as the executable name and
`/drive` as the output topic.

ros2 run team_driver driver
ros2 launch team_driver driver.launch.py

What you are allowed to read (see docs/06-rules.md):
    /scan               LiDAR, 819 beams over 270 degrees
    /ego_racecar/odom   ground-truth pose and velocity - ALLOWED and RECOMMENDED
    TF, /map            the static map
What you publish:
    /drive              AckermannDriveStamped
    /driver/path        nav_msgs/Path for RViz
    /driver/markers     anything of your own for visualisation
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from .plans import compute_control, construct_path
from .state import DriverConfig, DriverState

# from sensor_msgs.msg import LaserScan


class Driver(Node):

    def __init__(self):
        super().__init__("driver")

        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("drive_topic", "/drive")

        self.declare_parameter("max_speed", 6.0)
        self.declare_parameter("min_speed", 1.0)

        self.declare_parameter("wheelbase", 0.33)
        self.declare_parameter("max_steering", 0.45)

        self.declare_parameter("min_lookahead", 0.45)
        self.declare_parameter("max_lookahead", 2.0)

        self.declare_parameter("lookahead_speed_gain", 0.25)
        self.declare_parameter("lookahead_steering_reduction", 0.70)

        self.declare_parameter("steering_speed_reduction", 0.75)

        self.config = DriverConfig(
            max_speed=self.get_parameter("max_speed").value,
            min_speed=self.get_parameter("min_speed").value,
            wheelbase=self.get_parameter("wheelbase").value,
            max_steering=self.get_parameter("max_steering").value,
            min_lookahead=self.get_parameter("min_lookahead").value,
            max_lookahead=self.get_parameter("max_lookahead").value,
            lookahead_speed_gain=self.get_parameter("lookahead_speed_gain").value,
            lookahead_steering_reduction=self.get_parameter(
                "lookahead_steering_reduction"
            ).value,
            steering_speed_reduction=self.get_parameter(
                "steering_speed_reduction"
            ).value,
        )

        self.state = DriverState()

        self.previous_time = None
        self.previous_speed = None

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.get_parameter("drive_topic").value,
            10,
        )

        self.path_pub = self.create_publisher(
            Path,
            "/driver/path",
            1,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/driver/markers",
            1,
        )

        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            10,
        )

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.map_callback,
            map_qos,
        )

        # Keep LiDAR disabled for now.
        #
        # self.create_subscription(
        #     LaserScan,
        #     "/scan",
        #     self.scan_callback,
        #     10,
        # )

        self._marker_divisor = 0

        self.get_logger().info("team_driver is up")

    def odom_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.previous_time is not None:
            dt = current_time - self.previous_time

            if dt > 0.0:
                self.state.dt = dt
            else:
                self.get_logger().warn(f"Invalid odom dt: {dt}")

        self.previous_time = current_time

        self.state.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

        q = msg.pose.pose.orientation

        self.state.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        current_speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )

        if (
            self.previous_speed is not None
            and self.state.dt is not None
            and self.state.dt > 0.0
        ):
            self.state.acceleration = (
                current_speed - self.previous_speed
            ) / self.state.dt

        self.state.speed = current_speed
        self.previous_speed = current_speed

        # Keep the simulator timestamp so RViz does not get
        # a future timestamp relative to the simulator TF.
        self.state.stamp = msg.header.stamp

        if self.state.map is not None and not self.state.path:
            self.state.path = construct_path(self.state)

            self.get_logger().info(f"Path constructed: {len(self.state.path)} points")

            self.publish_path()

        self.update_control()

    def map_callback(self, msg):
        if self.state.map is None:
            self.get_logger().info(
                f"MAP frame={msg.header.frame_id}, "
                f"size={msg.info.width}x{msg.info.height}, "
                f"resolution={msg.info.resolution}, "
                f"origin=({msg.info.origin.position.x}, "
                f"{msg.info.origin.position.y})"
            )

        self.state.map = msg

        if self.state.position is not None and not self.state.path:
            self.state.path = construct_path(self.state)

            self.get_logger().info(f"Path constructed: {len(self.state.path)} points")

            self.publish_path()

    def update_control(self):
        if self.state.position is None:
            self.get_logger().warn("No odometry position yet")
            return

        if self.state.map is None:
            self.get_logger().warn("No map received yet")
            return

        steering, speed = compute_control(
            self.state,
            self.config,
        )

        self.publish(steering, speed)

        self._marker_divisor = (self._marker_divisor + 1) % 10

        if self._marker_divisor == 0:
            self.publish_marker(steering)

    def publish(self, steering, speed):
        msg = AckermannDriveStamped()

        if self.state.stamp is not None:
            msg.header.stamp = self.state.stamp

        msg.drive.steering_angle = float(steering)
        msg.drive.speed = float(speed)

        self.drive_pub.publish(msg)

    def publish_path(self):
        if self.state.path is None:
            return

        path_msg = Path()

        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.state.stamp

        for x, y in self.state.path:
            pose = PoseStamped()

            pose.header.frame_id = "map"
            pose.header.stamp = self.state.stamp

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_marker(self, target_angle):
        """Draw where the car thinks it is going."""
        marker = Marker()

        marker.header.frame_id = "ego_racecar/base_link"
        marker.header.stamp = self.state.stamp

        marker.ns = "team_driver"
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 1.5
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 0.9

        marker.pose.orientation.z = math.sin(target_angle / 2.0)
        marker.pose.orientation.w = math.cos(target_angle / 2.0)

        array = MarkerArray()
        array.markers.append(marker)

        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)

    node = Driver()

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
