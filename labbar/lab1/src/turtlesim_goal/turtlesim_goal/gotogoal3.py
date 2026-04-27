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
        super().__init__("turtlesim_goal")

        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
        )

        # Subscriber for turtle pose
        self.pose_subscriber = self.create_subscription(
            Pose, "/turtle1/pose", self.update_pose, 10
        )

        # Waypoints parameter as string
        # Example:
        # --ros-args -p waypoints:="[[1.0, 2.0], [3.0, 4.0]]"
        self.declare_parameter("waypoints", "[]")

        # Turtlesim bounds
        self.min_x = 0.0
        self.max_x = 11.0
        self.min_y = 0.0
        self.max_y = 11.0

        # Movement parameters
        self.max_linear_speed = 1.0
        self.min_linear_speed = 0.1
        self.angular_speed_factor = 2.5
        self.distance_tolerance = 0.15

        # Pause at each waypoint
        self.pause_duration = 1.0  # seconds

        # State
        self.pose = Pose()
        self.goal_pose = Pose()
        self.moving_to_goal = False
        self.pose_received = False
        self.last_log_time = 0.0

        self.waiting_at_waypoint = False
        self.wait_start_time = 0.0

        # Waypoint list
        self.waypoints = []
        self.current_waypoint_index = 0

        # Control loop
        self.timer = self.create_timer(0.1, self.controller_callback)

        # Load waypoint parameter
        self.load_waypoints()

    def update_pose(self, data):
        """Store current turtle pose."""
        self.pose = data
        self.pose_received = True

    def load_waypoints(self):
        """Read and validate waypoints from ROS2 parameter."""
        raw_waypoints = self.get_parameter("waypoints").value

        try:
            # Convert the string into a real Python object
            parsed_waypoints = ast.literal_eval(raw_waypoints)
        except (ValueError, SyntaxError):
            self.get_logger().error(
                "Invalid format for 'waypoints'. Example:\n"
                '[[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]]'
            )
            raise ValueError("Could not parse waypoints parameter")

        # Save the parsed and validated data into self.waypoints
        self.waypoints = self.validate_waypoints(parsed_waypoints)

        if not self.waypoints:
            raise ValueError("No valid waypoints provided.")
        else:
            self.get_logger().info(
                f"Loaded {len(self.waypoints)} waypoint(s): {self.waypoints}"
            )

    def validate_waypoints(self, waypoints):
        """
        Validate that waypoints is a list of [x, y] pairs,
        where x and y are numbers within turtlesim bounds.
        """
        if not isinstance(waypoints, list):
            raise ValueError("Waypoints must be a list.")

        validated = []

        for i, point in enumerate(waypoints):
            # Check that each waypoint is a list or tuple
            if not isinstance(point, (list, tuple)):
                raise ValueError(
                    f"Waypoint {i} is invalid. Each waypoint must be a list like [x, y]."
                )

            # Check that each waypoint contains exactly two values
            if len(point) != 2:
                raise ValueError(
                    f"Waypoint {i} is invalid. Each waypoint must contain exactly 2 values."
                )

            x, y = point“hela rundan” / the whole route is th

            # Check that both coordinates are numeric
            if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
                raise ValueError(
                    f"Waypoint {i} is invalid. Coordinates must be numbers, got {point}."
                )

            # Convert both values to floats
            x = float(x)
            y = float(y)

            # Boundary check: waypoint must stay inside turtlesim window
            if not (self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y):
                raise ValueError(
                    f"Waypoint {i} = [{x}, {y}] is outside turtlesim bounds "
                    f"([{self.min_x}, {self.max_x}] for x, [{self.min_y}, {self.max_y}] for y)."
                )

            validated.append([x, y])

        return validated

    def start_next_waypoint(self):
        """Start moving to the next waypoint."""
        # Check whether all waypoints have already been completed
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed.")
            self.moving_to_goal = False
            self.stop_turtle()
            return

        # Get the coordinates of the current waypoint
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
        """Calculate forward speed with stronger deceleration near goal."""
        distance = self.euclidean_distance()
        decel_zone = 1.0

        if distance < decel_zone:
            speed = self.max_linear_speed * (distance / decel_zone)
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
        angular_speed = angle_diff * self.angular_speed_factor

        # Clamp turning speed a bit for stability
        max_angular_speed = 2.0
        if angular_speed > max_angular_speed:
            angular_speed = max_angular_speed
        elif angular_speed < -max_angular_speed:
            angular_speed = -max_angular_speed

        return angular_speed

    def controller_callback(self):
        """Main control loop."""
        if not self.pose_received:
            return

        # Start first waypoint automatically
        if (
            not self.moving_to_goal
            and not self.waiting_at_waypoint
            and self.current_waypoint_index == 0
            and self.waypoints
        ):
            self.start_next_waypoint()
            return

        # Pause at waypoint before moving on
        if self.waiting_at_waypoint:
            self.stop_turtle()
            if time.time() - self.wait_start_time >= self.pause_duration:
                self.waiting_at_waypoint = False
                self.current_waypoint_index += 1
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

        # Goal reached
        if self.euclidean_distance() < self.distance_tolerance:
            self.stop_turtle()
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_index + 1} reached. Waiting..."
            )
            self.moving_to_goal = False
            self.waiting_at_waypoint = True
            self.wait_start_time = time.time()
            return

        vel_msg = Twist()

        angle_error = self.normalize_angle(
            self.calculate_steering_angle() - self.pose.theta
        )
        angular_diff = abs(angle_error)

        linear_velocity = self.calculate_linear_velocity()

        # Rotate in place first if the turtle is facing too far away
        if angular_diff > 0.4:
            vel_msg.linear.x = 0.0
        else:
            vel_msg.linear.x = linear_velocity * max(0.3, 1.0 - angular_diff)

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