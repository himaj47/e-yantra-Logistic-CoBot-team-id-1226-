#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes orientation data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###


'''
# Team ID:           LB#1226 
# Theme:             Cosmo Logistic 
# Author List:       Prathmesh Atkale 
# Filename:          ebot_docking_task5.py 
# Functions:         calculate_angular_error, calculate_linear_error, controller_loop, dock_control_callback , main

'''

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
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
        self.wall_allign=False 
        self.desired_yaw = 0.0  # Placeholder for target yaw

        # Subscribe to odometry and sensor data
       
        ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback, 10)
        self.odom_sub = self.create_subscription(Float32, '/orientation', self.odometry_callback, 10)
        
        # Create a service for docking control
        self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.robot_orient = None  # Initialize as None
        self.alpha=0.8

        # Internal state variables
        self.is_docking = False
        self.docking_complete = False
        self.robot_pose = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.usrleft_value = float('inf')  # Initialize sensor readings
        self.usrright_value = float('inf')

        # Timer for the control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)
        
    # callback for ultrasonic subscription
    def ultra_callback(self,msg:Float32MultiArray):
        self.usrleft_value = msg.data[4]
        self.usrright_value = msg.data[5]
        self.usrleft_value /=100  # Convert to cm  as above reading are in meters
        self.usrright_value /=100
        
    

    def odometry_callback(self, msg:Float32):
        if self.robot_orient is None:  # Initialize on first callback
            self.robot_orient = msg.data
        else:
            self.robot_orient = self.alpha * msg.data + (1 - self.alpha) * self.robot_orient

        print(self.robot_orient)  # Print the filtered orientation
       
    
    def avoid_collision_dock(self,Kp_angular , allign_error):
        left_ur=self.usrleft_value
        right_ur=self.usrright_value
        error=(left_ur-right_ur)
        if(abs(error)>allign_error):
            angular_vel=Kp_angular*error
            print(f'Avoiding Wall')
        else:
            print(f'No Collision ....')
            angular_vel=0.0
            self.wall_allign=True 

        return angular_vel

    def calculate_angular_error(self, Kp_angular, Kd_angular, desired_yaw, min_orient_error, prev_angular_error):
        '''
        Purpose:
        ---
        This function calculates the angular velocity required for the robot to align its orientation 
        with a desired yaw angle. It uses a Proportional-Derivative (PD) control mechanism to determine 
        the angular velocity and updates the angular error for feedback control.

        Input Arguments:
        ---
        `Kp_angular` : [float]
            The proportional gain for angular velocity control.

        `Kd_angular` : [float]
            The derivative gain for angular velocity control.

        `desired_yaw` : [float]
            The target yaw angle (in radians) that the robot needs to align with.

        `min_orient_error` : [float]
            The minimum threshold for the angular error (in radians) below which the robot is 
            considered aligned.

        `prev_angular_error` : [float]
            The angular error (in radians) from the previous time step, used to compute 
            the derivative term.

        Returns:
        ---
        `angular_vel` : [float]
            The calculated angular velocity for the robot to align its orientation.

        `angular_error` : [float]
            The current angular error (in radians), normalized to the range [-π, π].

        Example call:
        ---
        angular_vel, angular_error = self.calculate_angular_error(
            Kp_angular=3.5, 
            Kd_angular=0.001, 
            desired_yaw=1.57, 
            min_orient_error=0.05, 
            prev_angular_error=0.1
        )
        '''

        # Calculate angular error and normalize it
        
        

        #for Odometry hardware
        angular_error=(desired_yaw-self.robot_orient)
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))  # Normalize to [-pi, pi]

        # Calculate derivative of the angular error
        angular_error_derivative = angular_error - prev_angular_error
         
        prev_angular_error=angular_error
        if abs(angular_error) > min_orient_error:
            # PD control
            angular_vel = Kp_angular * angular_error + Kd_angular * angular_error_derivative
            # self.get_logger().info(f"Angular error (normalized): {angular_error} | Derivative: {angular_error_derivative}")
        else:
            angular_vel = 0.0
            self.is_angular_aligned = True  # Mark angular alignment as complete
            print('Alligned with the oreintation')
        return angular_vel, angular_error

    def calculate_linear_error(self, Kp_linear, safe_distance):
        '''
        Purpose:
        ---
        This function calculates the linear velocity required for the robot to maintain a safe 
        distance from an object or target using Proportional-Derivative (PD) control. It uses 
        sensor data to compute the distance error and adjusts the velocity accordingly.

        Input Arguments:
        ---
        `Kp_linear` : [float]
            The proportional gain for linear velocity control.

        `safe_distance` : [float]
            The target safe distance (in meters) that the robot should maintain from the object.

        Returns:
        ---
        `linear_vel` : [float]
            The calculated linear velocity to reduce the distance error.

        Example call:
        ---
        linear_vel = self.calculate_linear_error(Kp_linear=3.2, safe_distance=0.5)
        '''

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
        '''
        Purpose:
        ---
        The `controller_loop` function manages the docking procedure of the robot. It performs angular alignment first 
        and, once aligned, proceeds with linear alignment using Proportional-Derivative (PD) control. It generates 
        and publishes velocity commands to guide the robot toward its target position.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.controller_loop()
        '''
        if not self.is_docking:
            return

        # Control parameters
        safe_distance = self.safe_dist  # 0.06
        min_orient_error = 0.05  # Adjusted for finer angular precision
        kp_linear = 1.5  # Proportional gain for linear control
        kp_angular =-1.3  # Proportional gain for angular control
        kd_angular = -0.0002 # Derivative gain for angular control
        max_linear_vel = 0.8
        max_angular_vel = 0.8
        Kp_ang_wall=-2.0
        min_error_wall=0.005

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
        print('alligned.........')
        if not self.wall_allign:
            twist_angular=Twist()
            twist_angular.angular.z=(self.avoid_collision_dock(Kp_ang_wall,min_error_wall))
            self.cmd_vel_pub.publish(twist_angular)

            return
       
        angular_vel = 0.0  # Stop angular correction once aligned
        self.linear_error = self.calculate_linear_error(kp_linear, safe_distance)

        # Generate and publish velocity commands for linear movement
        twist_msg = Twist()
        twist_msg.angular.z = angular_vel
        twist_msg.linear.x = -min(self.linear_error, max_linear_vel) 
        self.cmd_vel_pub.publish(twist_msg)


    def dock_control_callback(self, request, response):
        '''
        Purpose:
        ---
        The `dock_control_callback` function handles the docking request from the client. It sets the desired yaw and safe 
        distance for docking, then starts the docking procedure by aligning the robot both angularly and linearly. 
        It returns the result of the docking operation in the response.

        Input Arguments:
        ---
        `request` :  [ object ]
            The request object containing the desired orientation (`request.orientation`) and safe distance (`request.distance`) 
            for docking.

        `response` :  [ object ]
            The response object that will be populated with the result of the docking operation (`success` and `message` fields).

        Returns:
        ---
        `response` :  [ object ]
            The response object, with fields indicating whether the docking was successful and a message.

        Example call:
        ---
        request = DockSw.Request()
        request.distance =0.5
        request.orientation=1.57 
        response = DockSw.Response()
        result = self.dock_control_callback(request, response)
        '''
    

        self.desired_yaw = request.orientation
        self.safe_dist=request.distance
        self.is_angular_aligned = False
        self.wall_allign=False 
        self.is_docking = True
        self.docking_complete = False

        # Wait for docking completion
        rate = self.create_rate(100, self.get_clock())
        while not self.docking_complete:
            # self.get_logger().info("Docking in progress...")
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
