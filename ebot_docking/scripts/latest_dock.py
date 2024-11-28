#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math

class MyRobotDockingController(Node):

    def __init__(self):
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()
        self.previous_dis = 0.0
        self.prev_angular_error = 0.0
        self.is_angular_aligned = False  # Initialize angular alignment flag
        self.desired_yaw = 0.0  # Placeholder for target yaw

        # Subscribe to odometry and sensor data
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Create a service for docking control
        self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state variables
        self.is_docking = False
        self.docking_complete = False
        self.robot_pose = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.usrleft_value = float('inf')  # Initialize sensor readings
        self.usrright_value = float('inf')

        # Timer for the control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    def odometry_callback(self, msg):
        # Update robot pose from odometry data
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range

    def calculate_angular_error(self, Kp_angular, Kd_angular, desired_yaw, min_orient_error, prev_angular_error):
        # Calculate angular error and normalize it
        angular_error = (desired_yaw - self.robot_pose[2])-3.24  
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))  # Normalize to [-pi, pi]

        # Calculate derivative of the angular error
        angular_error_derivative = angular_error - prev_angular_error

        if abs(angular_error) > min_orient_error:
            # PD control
            angular_vel = Kp_angular * angular_error + Kd_angular * angular_error_derivative
            self.get_logger().info(f"Angular error (normalized): {angular_error} | Derivative: {angular_error_derivative}")
        else:
            angular_vel = 0.0
            self.is_angular_aligned = True  # Mark angular alignment as complete

        return angular_vel, angular_error

    def calculate_linear_error(self, Kp_linear, safe_distance):
        # Calculate distance error from ultrasonic sensors
        distance_error = (self.usrleft_value + self.usrright_value) / 2
        delta_distance = distance_error - self.previous_dis
        self.previous_dis = distance_error

        if distance_error > safe_distance:
            linear_vel = Kp_linear * distance_error - 0.1 * delta_distance
            self.get_logger().info(f"Distance error: {distance_error}")
        else:
            linear_vel = 0.0
            self.docking_complete = True
            self.get_logger().info("Docking complete.")
            self.is_docking = False

        return linear_vel

    def controller_loop(self):
        if not self.is_docking:
            return

        # Control parameters
        safe_distance = 0.06
        min_orient_error = 0.05  # Adjusted for finer angular precision
        kp_linear = 1.9  # Proportional gain for linear control
        kp_angular = 1.6  # Proportional gain for angular control
        kd_angular = 0.0001  # Derivative gain for angular control
        max_linear_vel = 0.8
        max_angular_vel = 0.8

        # Angular alignment logic
        if not self.is_angular_aligned:
            # Calculate angular error using yaw from odometry if not already aligned
            angular_vel, angular_error = self.calculate_angular_error(
                kp_angular, kd_angular, self.desired_yaw, min_orient_error, self.prev_angular_error
            )
            # Store the current angular error for the next loop iteration
            self.prev_angular_error = angular_error

            # Generate and publish only angular velocity if not aligned
            twist_msg = Twist()
            twist_msg.angular.z = min(angular_vel, max_angular_vel)
            twist_msg.linear.x = 0.0  # Prevent linear movement until angular alignment is done
            self.cmd_vel_pub.publish(twist_msg)
            
            # Return early to avoid engaging linear control
            return

        # Linear alignment logic - Only reached if angular alignment is complete
        angular_vel = 0.0  # Stop angular correction once aligned
        self.linear_error = self.calculate_linear_error(kp_linear, safe_distance)

        # Generate and publish velocity commands for linear movement
        twist_msg = Twist()
        twist_msg.angular.z = angular_vel
        twist_msg.linear.x = -min(self.linear_error, max_linear_vel) 
        self.cmd_vel_pub.publish(twist_msg)


    def dock_control_callback(self, request, response):
        self.desired_yaw = request.orientation
        self.is_angular_aligned = False
        self.is_docking = True
        self.docking_complete = False

        # Wait for docking completion
        rate = self.create_rate(100, self.get_clock())
        while not self.docking_complete:
            self.get_logger().info("Docking in progress...")
            rate.sleep()

        response.success = True
        response.message = "Docking completed"
        return response


def main(args=None):
    rclpy.init(args=args)
    my_robot_docking_controller = MyRobotDockingController()
    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)
    executor.spin()
    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
