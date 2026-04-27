#!/usr/bin/env python3
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32


class Talker1(Node):
    def __init__(self):
        super().__init__("talker1")
        self.pub = self.create_publisher(Int32, "chatter1", 10)
        self.tmr = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Int32()
        msg.data = random.randint(0, 100)
        self.get_logger().info(f"Publishing random integer: %d {msg.data}")
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = Talker1()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()