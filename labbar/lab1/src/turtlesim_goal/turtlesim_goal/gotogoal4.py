#!/usr/bin/env python3
import ast
import time
from math import sqrt, atan2, pi

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleBot(Node):
    def __init__(self):
        super().__init__(
            "turtlesim_goal",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )

        # Subscriber for turtle pose
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.update_pose, 10
        )

        # Turtlesim window bounds
        self.min_x = 0.0
        self.max_x = 11.0
        self.min_y = 0.0
        self.max_y = 11.0

        # Movement parameters
        self.max_linear_speed = 1.5
        self.min_linear_speed = 0.3
        self.angular_speed_factor = 4.0
        self.distance_tolerance = 0.1

        # State
        self.pose = Pose()
        self.goal_pose = Pose()
        self.moving_to_goal = False
        self.last_log_time = 0.0
        self.pose_received = False

        # Waypoints
        self.waypoints = []
        self.current_waypoint_index = 0

        # Load parameter
        self.load_waypoints()

        # Control loop
        self.timer = self.create_timer(0.1, self.controller_callback)

    def update_pose(self, data):
        """Store the turtle's current position."""
        self.pose = data
        self.pose_received = True

    def load_waypoints(self):
        """Read and validate waypoints from ROS2 parameter."""
        if not self.has_parameter("waypoints"):
            self.get_logger().warn("No 'waypoints' parameter provided.")
            self.waypoints = []
            return

        raw_waypoints = self.get_parameter("waypoints").value
        self.get_logger().info(
            f"Raw waypoints parameter: {raw_waypoints} (type: {type(raw_waypoints).__name__})"
        )

        parsed_waypoints = self.parse_waypoints(raw_waypoints)
        self.waypoints = self.validate_waypoints(parsed_waypoints)

        if not self.waypoints:
            self.get_logger().warn("No valid waypoints provided.")
        else:
            self.get_logger().info(
                f"Loaded {len(self.waypoints)} waypoint(s): {self.waypoints}"
            )

    def parse_waypoints(self, raw_waypoints):
        """
        Accept:
        1. String: '[[1.0, 2.0], [3.0, 4.0]]'
        2. Flat numeric list: [1.0, 2.0, 3.0, 4.0]
        3. Nested list/tuple: [[1.0, 2.0], [3.0, 4.0]]
        """
        # Case 1: parameter came in as a string
        if isinstance(raw_waypoints, str):
            try:
                parsed = ast.literal_eval(raw_waypoints)
            except (ValueError, SyntaxError):
                raise ValueError(
                    "Invalid 'waypoints' string format. Example: [[1.0, 2.0], [3.0, 4.0]]"
                )
            return parsed

        # Case 2: flat numeric list
        if isinstance(raw_waypoints, (list, tuple)) and all(
            isinstance(v, (int, float)) for v in raw_waypoints
        ):
            if len(raw_waypoints) % 2 != 0:
                raise ValueError(
                    "Waypoints list has an odd number of values. Coordinates must come in x,y pairs."
                )

            paired = []
            for i in range(0, len(raw_waypoints), 2):
                paired.append([float(raw_waypoints[i]), float(raw_waypoints[i + 1])])
            return paired

        # Case 3: nested list/tuple
        if isinstance(raw_waypoints, (list, tuple)):
            return list(raw_waypoints)

        raise ValueError(
            f"Unsupported parameter type for waypoints: {type(raw_waypoints).__name__}"
        )

    def validate_waypoints(self, waypoints):
        """Validate waypoints as [[x, y], [x, y], ...]."""
        if not isinstance(waypoints, (list, tuple)):
            raise ValueError("Waypoints must be a list.")

        validated = []

        for i, point in enumerate(waypoints):
            if not isinstance(point, (list, tuple)):
                raise ValueError(
                    f"Waypoint {i} is invalid. Each waypoint must be [x, y]."
                )

            if len(point) != 2:
                raise ValueError(
                    f"Waypoint {i} is invalid. Each waypoint must contain exactly 2 values."
                )

            x, y = point

            if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
                raise ValueError(
                    f"Waypoint {i} is invalid. Coordinates must be numbers, got {point}."
                )

            x = float(x)
            y = float(y)

            if not (self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y):
                raise ValueError(
                    f"Waypoint {i} = [{x}, {y}] is outside turtlesim bounds "
                    f"([{self.min_x}, {self.max_x}] for x, [{self.min_y}, {self.max_y}] for y)."
                )

            validated.append([x, y])

        return validated

    def start_next_waypoint(self):
        """Start moving to the next waypoint."""
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed.")
            self.moving_to_goal = False
            self.stop_turtle()
            return

        x, y = self.waypoints[self.current_waypoint_index]
        self.goal_pose.x = x
        self.goal_pose.y = y
        self.moving_to_goal = True

        self.get_logger().info(
            f"Moving to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: "
            f"x={x:.2f}, y={y:.2f}"
        )

    def stop_turtle(self):
        """Publish zero velocity."""
        vel_msg = Twist()
        self.velocity_publisher.publish(vel_msg)

    def euclidean_distance(self):
        """Distance between current position and goal."""
        return sqrt(
            (self.goal_pose.x - self.pose.x) ** 2
            + (self.goal_pose.y - self.pose.y) ** 2
        )

    def calculate_linear_velocity(self):
        """Calculate forward speed with deceleration near goal."""
        distance = self.euclidean_distance()
        decel_zone = self.distance_tolerance * 2.0

        if distance < decel_zone:
            speed = self.min_linear_speed + (
                self.max_linear_speed - self.min_linear_speed
            ) * (distance / decel_zone)
        else:
            speed = self.max_linear_speed

        return max(self.min_linear_speed, min(speed, self.max_linear_speed))

    def calculate_steering_angle(self):
        """Angle toward current goal."""
        return atan2(
            self.goal_pose.y - self.pose.y,
            self.goal_pose.x - self.pose.x
        )

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def calculate_angular_velocity(self):
        """Calculate rotational speed for steering."""
        angle_diff = self.normalize_angle(
            self.calculate_steering_angle() - self.pose.theta
        )

        if abs(angle_diff) < 0.1:
            return angle_diff * self.angular_speed_factor * 0.8
        elif abs(angle_diff) < 0.5:
            return angle_diff * self.angular_speed_factor
        else:
            return angle_diff * self.angular_speed_factor * 1.2

    def controller_callback(self):
        """Main control loop."""
        if not self.pose_received:
            return

        if not self.moving_to_goal and self.current_waypoint_index == 0 and self.waypoints:
            self.start_next_waypoint()
            return

        if not self.moving_to_goal:
            return

        current_time = time.time()
        if current_time - self.last_log_time >= 1.0:
            self.get_logger().info(
                f"Current position: x={self.pose.x:.2f}, y={self.pose.y:.2f}, "
                f"distance_to_goal={self.euclidean_distance():.2f}"
            )
            self.last_log_time = current_time

        if self.euclidean_distance() < self.distance_tolerance:
            self.stop_turtle()
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_index + 1} reached."
            )
            self.current_waypoint_index += 1
            self.moving_to_goal = False
            self.start_next_waypoint()
            return

        vel_msg = Twist()

        linear_velocity = self.calculate_linear_velocity()
        angular_diff = abs(
            self.normalize_angle(self.calculate_steering_angle() - self.pose.theta)
        )

        turn_factor = 1.0
        if angular_diff > 0.5:
            turn_factor = max(0.4, 1.0 - angular_diff / pi)

        vel_msg.linear.x = linear_velocity * turn_factor
        vel_msg.angular.z = self.calculate_angular_velocity()

        self.velocity_publisher.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        turtlebot = TurtleBot()
    except ValueError as e:
        print(f"Startup error: {e}")
        rclpy.shutdown()
        return

    try:
        rclpy.spin(turtlebot)
    except KeyboardInterrupt:
        pass
    finally:
        turtlebot.stop_turtle()
        turtlebot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()