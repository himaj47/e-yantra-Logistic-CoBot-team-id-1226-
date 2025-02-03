#!/usr/bin/env python3
'''
# Team ID:          [ LB#1226 ]
# Theme:            [ Cosmo Logistic ]
# Author List:      [ Prathmesh Atkale ]
# Filename:         [ ebot_nav2_cmd_task4c.py ]
# Functions:        [ create_goal_pose, initiate_payload_action ,box_payload,initiate_docking,set_initial_pose, recieve_pose, conveyor_pose, execute_navigation, main]
# Global variables: [passed_point,  total_box, receive_pos]
'''

################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node

from ebot_docking.srv import DockSw
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import time
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist

class NavigationDockingController(Node):
    def __init__(self):
        super().__init__('navigation_docking')
        self.navigator = BasicNavigator()

        # Initialize current position
        self.current_pose = None

        # Set up odometry subscription
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
        # Initialize docking service client
        self.docking_client = self.create_client(DockSw, '/dock_control')
        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for DockSw service...')
        
        
    #Function to have Current Robot Pose and Orientation
    def odometry_callback(self, msg):
        """Update the robot's current pose."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (position.x, position.y, yaw)
    
    

    #To calculate quaternion from the yaw
  

    def initiate_docking(self, target_distance, orientation_angle, rack_number):
        '''
        Purpose:
        ---
        This function initiates the docking process by sending a request to the docking service. 
        The process involves aligning the robot to a specific distance, angle, and rack number.

        Input Arguments:
        ---
        `target_distance` : [float]
            The distance (in meters) from the docking target that the robot needs to maintain.

        `orientation_angle` : [float]
            The desired orientation angle (in radians) for docking alignment.

        `rack_number` : [int]
            The target rack number where docking needs to occur.

        Returns:
        ---
        `success` : [bool]
            Indicates whether the docking process was successfully initiated and executed.

        Example call:
        ---
        success = self.initiate_docking(0.08, 1.57, 2)
        if success:
            print("Docking process completed successfully.")
        else:
            print("Docking process failed.")
        '''

        self.get_logger().info("Docking initiated and the request is being sent")

        docking_request = DockSw.Request()
        docking_request.startcmd = True  # Initiates the docking process
        docking_request.undocking = False  # Specifies that we are docking, not undocking
        docking_request.linear_dock = True  # Assuming you want to perform linear docking
        docking_request.orientation_dock = True # Set to True if orientation adjustment is needed
        docking_request.distance = target_distance
        docking_request.orientation = orientation_angle
        docking_request.rack_no = rack_number # Provide the target rack number

        future = self.docking_client.call_async(docking_request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Docking successful with message: {future.result().message}")
            return True
        else:
            self.get_logger().error('Docking service call failed')
            return False

    def execute_navigation(self):
        '''
        Purpose:
        ---
        This function executes the navigation task by navigating the robot through predefined waypoints. 
        It manages payload  drop actions, docking operations, and monitors the task status 
        to ensure successful execution.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.execute_navigation()
        '''

        """Run the navigation task and manage docking and payload services at specific waypoints."""
       
        
        self.get_logger().info(f'Task Completed SuccessFully...')
        # yaw=euler_from_quaternion([0.5778,-0.4822,0.0886,0.6526])
        # sec=euler_from_quaternion([0.000,0.000,0.0289,-0.9996])
        # print(sec)
        self.initiate_docking(target_distance=0.70 ,orientation_angle=0.00,rack_number='')
##################### MAIN FUNCTION #######################


def main(args=None):
    rclpy.init(args=args)
    navigation_docking_controller = NavigationDockingController()
    navigation_docking_controller.execute_navigation()
    navigation_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()