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

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Create a ROS2 service for controlling docking behavior
        self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialized flags and parameters
        self.is_docking = False
        self.docking_complete = False  # New flag to track docking completion
        self.robot_pose = [0.0, 0.0, 0.0]
        self.usrleft_value = float('inf')  # Initialize sensor readings
        self.usrright_value = float('inf')

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    # Callback function for the right ultrasonic sensor
    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range
    
    #Function to Calculate Angular Error
    def  calculate_angular_error(self,Kp_angular,min_orient_error):
        alignment_error = self.usrleft_value - self.usrright_value
        if abs(alignment_error) > min_orient_error:
            angular_vel = Kp_angular * alignment_error
            self.get_logger().info(f"Approaching target, angualr error: {alignment_error}")   
        else:
            angular_vel=0.0            
        return angular_vel
    
     #Function to Calculate linear Error
    def calculate_linear_error(self,Kp_linear,safe_distance):
        distance_error = (self.usrleft_value+self.usrright_value)/2
        delta_distance = distance_error - self.previous_dis
        self.previous_dis = distance_error

        if distance_error > safe_distance:
            linear_vel = Kp_linear * distance_error - 0.1 * delta_distance
            self.get_logger().info(f"Approaching target, distance error: {distance_error}")
        else:
            linear_vel=0.0
            self.docking_complete=True
            self.get_logger().info(f'Aligned Successfully')
            self.is_docking=False
        return linear_vel

    def controller_loop(self):
        if not self.is_docking:
            return
        safe_distance=0.08
        min_orient_error=0.01
        kp_linear = 2.1    #Proportional for linear
        kp_angular = 1.8    #Proportional for angular
        max_linear_vel = 0.6
        max_angular_vel = 0.6
        
        # Get the angular error
        angualr_error=self.calculate_angular_error(kp_angular,min_orient_error)
        #Get the linear error
        linear_error=self.calculate_linear_error(kp_linear,safe_distance)
       
        twist_msg = Twist()
        twist_msg.linear.x = -min(linear_error,max_linear_vel)
        twist_msg.angular.z = -min(angualr_error,max_angular_vel)
        self.cmd_vel_pub.publish(twist_msg)

    def dock_control_callback(self, request, response):
       
        rate=self.create_rate(10,self.get_clock())
        # Start the docking process  
        self.is_docking = True  # Initiating docking action
        self.docking_complete = False  #  completion flag is reset to false

        while not self.docking_complete:
            self.get_logger().info("Waiting for docking to complete")
            rate.sleep()

        response.success = True
        response.message = "Docking completed"
        
        return response

# Main function to initialize the ROS2 node and spin the executor
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


