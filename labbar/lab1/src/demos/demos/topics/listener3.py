#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class Listener2(Node):
    def __init__(self):
        super().__init__("listener2")

        self.sub2 = self.create_subscription(
            String, "chatter2", self.chatter2_callback, 10
        )

    def chatter2_callback(self, msg):
        self.get_logger().info(f"I heard from talker2: [{msg.data}]")


def main(args=None):
    rclpy.init(args=args)

    node = Listener2()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()