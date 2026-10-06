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
    /drive
    /driver/path
    /driver/connected_cone_map
    /driver/track_corridor_visual
    /driver/markers
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from .control import LQRTracker
from .path import construct_path
from .state import DriverState


class Driver(Node):

    def __init__(self):
        super().__init__("driver")

        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("drive_topic", "/drive")

        # Robot
        self.declare_parameter("robot.lf", 0.15875)
        self.declare_parameter("robot.lr", 0.17145)
        self.declare_parameter("robot.width", 0.31)
        self.declare_parameter("robot.length", 0.58)
        self.declare_parameter("robot.mass", 3.74)
        self.declare_parameter("robot.inertia_z", 0.04712)
        self.declare_parameter("robot.cg_height", 0.074)
        self.declare_parameter("robot.friction", 1.0489)
        self.declare_parameter("robot.cornering_stiffness_front", 4.718)
        self.declare_parameter("robot.cornering_stiffness_rear", 5.4562)
        self.declare_parameter("robot.steering_min", -0.4189)
        self.declare_parameter("robot.steering_max", 0.4189)
        self.declare_parameter("robot.steering_rate_min", -3.2)
        self.declare_parameter("robot.steering_rate_max", 3.2)
        self.declare_parameter("robot.acceleration_min", -9.51)
        self.declare_parameter("robot.acceleration_max", 9.51)
        self.declare_parameter("robot.velocity_min", -5.0)
        self.declare_parameter("robot.velocity_max", 20.0)
        self.declare_parameter("robot.dynamic_switch_speed", 0.5)

        # LQR
        self.declare_parameter("lqr.lateral_weight", 1.0)
        self.declare_parameter("lqr.heading_weight", 1.0)
        self.declare_parameter("lqr.steering_weight", 0.1)
        self.declare_parameter("lqr.max_speed", 6.0)
        self.declare_parameter("lqr.min_speed", 0.2)
        self.declare_parameter("lqr.max_lateral_acceleration", 2.0)
        self.declare_parameter("lqr.lookahead_distance", 0.5)

        # Read robot configuration.
        robot = {
            "lf": self.get_parameter("robot.lf").value,
            "lr": self.get_parameter("robot.lr").value,
            "width": self.get_parameter("robot.width").value,
            "length": self.get_parameter("robot.length").value,
            "mass": self.get_parameter("robot.mass").value,
            "inertia_z": self.get_parameter("robot.inertia_z").value,
            "cg_height": self.get_parameter("robot.cg_height").value,
            "friction": self.get_parameter("robot.friction").value,
            "cornering_stiffness_front": self.get_parameter(
                "robot.cornering_stiffness_front"
            ).value,
            "cornering_stiffness_rear": self.get_parameter(
                "robot.cornering_stiffness_rear"
            ).value,
            "steering_min": self.get_parameter("robot.steering_min").value,
            "steering_max": self.get_parameter("robot.steering_max").value,
            "steering_rate_min": self.get_parameter("robot.steering_rate_min").value,
            "steering_rate_max": self.get_parameter("robot.steering_rate_max").value,
            "acceleration_min": self.get_parameter("robot.acceleration_min").value,
            "acceleration_max": self.get_parameter("robot.acceleration_max").value,
            "velocity_min": self.get_parameter("robot.velocity_min").value,
            "velocity_max": self.get_parameter("robot.velocity_max").value,
            "dynamic_switch_speed": self.get_parameter(
                "robot.dynamic_switch_speed"
            ).value,
        }

        # Create LQR.
        self.controller = LQRTracker(
            wheelbase=robot["lf"] + robot["lr"],
            max_steering=self.get_parameter("robot.steering_max").value,
            max_speed=self.get_parameter("lqr.max_speed").value,
            min_speed=self.get_parameter("lqr.min_speed").value,
            lateral_weight=self.get_parameter("lqr.lateral_weight").value,
            heading_weight=self.get_parameter("lqr.heading_weight").value,
            steering_weight=self.get_parameter("lqr.steering_weight").value,
            max_lateral_acceleration=self.get_parameter(
                "lqr.max_lateral_acceleration"
            ).value,
            lookahead_distance=self.get_parameter("lqr.lookahead_distance").value,
        )

        self.state = DriverState()

        self.previous_time = None
        self.previous_speed = None

        # Publishers

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
        self.connected_cone_map_pub = self.create_publisher(
            OccupancyGrid,
            "/driver/connected_cone_map",
            1,
        )
        self.corridor_pub = self.create_publisher(
            Marker,
            "/driver/track_corridor_visual",
            1,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/driver/markers",
            1,
        )

        # Subscribers

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

        self._marker_divisor = 0

        self.get_logger().info("team_driver is up")

    def odom_callback(self, msg):
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.previous_time is not None:
            dt = current_time - self.previous_time

            if dt > 0.0:
                self.state.dt = dt

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

        self.state.yaw_rate = msg.twist.twist.angular.z

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y

        if abs(vx) + abs(vy) > 1e-6:
            self.state.beta = math.atan2(
                vy,
                vx,
            )
        else:
            self.state.beta = 0.0

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

        self.state.stamp = msg.header.stamp

        if self.state.map is not None and not self.state.path:
            self.state.path = construct_path(
                self.state,
                self.connected_cone_map_pub,
                self.corridor_pub,
            )

            self.get_logger().info(f"Path constructed: {len(self.state.path)} points")

            self.publish_path()

        self.update_control()

    def map_callback(self, msg):
        if self.state.map is None:
            self.get_logger().info(
                f"MAP frame={msg.header.frame_id}, "
                f"size={msg.info.width}x{msg.info.height}, "
                f"resolution={msg.info.resolution}, "
                f"origin=("
                f"{msg.info.origin.position.x}, "
                f"{msg.info.origin.position.y}"
                f")"
            )

        self.state.map = msg

        if self.state.position is not None and not self.state.path:
            self.state.path = construct_path(
                self.state,
                self.connected_cone_map_pub,
                self.corridor_pub,
            )

            self.get_logger().info(f"Path constructed: {len(self.state.path)} points")

            self.publish_path()

    def update_control(self):
        if self.state.position is None:
            return

        if self.state.map is None:
            return

        if not self.state.path:
            return

        steering, speed = self.controller.control(
            self.state,
            self.state.path,
            self.state.corridor,
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
