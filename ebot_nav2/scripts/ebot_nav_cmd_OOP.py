#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

class NavCmd(Node):
    def __init__(self):
        super().__init__("ebot_nav2_cmd")


def main(args=None):
    rclpy.init(args=args)

    rclpy.shutdown()

if __name__ == "__main__":
    main()