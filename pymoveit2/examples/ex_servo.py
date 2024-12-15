#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion.
`ros2 run pymoveit2 ex_servo.py`
"""


from math import cos, sin
import math, time
from copy import deepcopy
import rclpy
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from pymoveit2 import MoveIt2Servo
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# Initialize message based on passed arguments
linear_speed = 0.3
angular_speed = 0.3

__twist_msg = TwistStamped()
__twist_msg.header.frame_id = ur5.base_link_name()
__twist_msg.twist.linear.x = linear_speed
__twist_msg.twist.linear.y = linear_speed
__twist_msg.twist.linear.z = linear_speed
__twist_msg.twist.angular.x = angular_speed
__twist_msg.twist.angular.y = angular_speed
__twist_msg.twist.angular.z = angular_speed

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()
    __twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    moveit2_servo = MoveIt2Servo(
        node=node,
        frame_id=ur5.base_link_name(),
        callback_group=callback_group,
    )
    
    def servo_circular_motion():
        """Move in a circular motion using Servo"""
        now_sec = node.get_clock().now().nanoseconds * 1e-9
        # moveit2_servo(linear=(sin(now_sec), cos(now_sec), 0.0), angular=(0.0, 0.0, 0.0))
        moveit2_servo(linear=(0.0, sin(now_sec)/7, cos(now_sec)/7), angular=(0.0, 0.0, 0.0))
        # Create timer for moving in a circular motion
    node.create_timer(0.02, servo_circular_motion)

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
