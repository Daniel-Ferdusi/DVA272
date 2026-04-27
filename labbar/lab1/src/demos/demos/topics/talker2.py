#!/usr/bin/env python3
import random
import string

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class Talker2(Node):
    def __init__(self):
        super().__init__("talker2")
        self.pub = self.create_publisher(String, "chatter2", 10)
        self.tmr = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = random.choice(string.ascii_lowercase)
        self.get_logger().info(f'Publishing random character: "{msg.data}"')
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Talker2()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()