#!/usr/bin/env python3
'''
# Team ID:          [ LB#1226 ]
# Theme:            [ Cosmo Logistic ]
# Author List:      [ Prathmesh Atkale ]
# Filename:         [ ebot_nav2_task4b.py ]
# Functions:        [ create_goal_pose ,initiate_docking,set_initial_pose, execute_navigation, main]
# Global variables: [passed_point]
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
from std_msgs.msg import Float32


class NavigationDockingController(Node):
    def __init__(self):
        super().__init__('navigation_docking_controller')
        self.navigator = BasicNavigator()

        # Initialize current position
        self.current_pose = None

        # Set up odometry subscription
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        global pre_goal
        self.arm_pose = [
            self.create_goal_pose(2.80, -2.65, -1.57),  # recieve pose  0.95, -2.65, 1.87
            self.create_goal_pose(2.80, -2.65, -1.57),  # recieve pose  0.95, -2.65, 1.87
        
        ]
        
        
        self.conveyor2_waypoint = [
            self.create_goal_pose(2.97, 1.84, 1.57),  # Conveyor 2  2.42,  2.55, -1.57
            self.create_goal_pose(2.97, 1.84, 1.57),  # Conveyor 2
           
        ]
                                                                                                                     
        self.conveyor1_waypoint=[

            self.create_goal_pose(1.95,  -1.23, 1.57),  # Conveyor 1  -4.4,  2.89, -1.57
            self.create_goal_pose(1.95,  -1.23, 1.57),  # Conveyor 1
           
        ]
        # Flags to ensure each action is triggered only once
        self.actions_triggered = [False, False,False,False,False,False]  # One per waypoint
        self.docking_in_progress = False  # Flag to track docking status
        
       
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
    

    
    def create_goal_pose(self, x, y, yaw):
        '''
        Purpose:
        ---
        This function creates a `PoseStamped` object to define a goal position and orientation for a robot 
        in a map frame, using the provided x, y coordinates and yaw (orientation angle).


        Input Arguments:
        ---
        `x` : [float]
        The x-coordinate of the goal position in the map frame.

        `y` : [float]
        The y-coordinate of the goal position in the map frame.

        `yaw` : [float]
        The orientation of the goal in radians, representing the rotation around the Z-axis.

        Returns:
        ---
        `pose` : [PoseStamped]
        A ROS2 PoseStamped object containing the goal position and orientation in the map frame.

        Example call:
        ---
        goal_pose = self.create_goal_pose(2.72, 2.88, -1.27)

        '''

        """Create a PoseStamped goal."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x   # Set X Coordinate 
        pose.pose.position.y = y   # Set Y Coordinate 

        #Function to have quaternion from yaw 
        q = self.yaw_to_quaternion(yaw)  
        pose.pose.orientation.x = q[0]  #Quaternion Component
        pose.pose.orientation.y = q[1] 
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
    

    #To calculate quaternion from the yaw
    def yaw_to_quaternion(self, yaw):
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    
    
       
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

    def set_initial_pose(self):
        '''
        Purpose:
        ---
        This function sets the initial pose of the robot in the map frame. The pose includes the robot's 
        position and orientation, which are necessary for localization and navigation.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.set_initial_pose()
        '''

        """Set the initial pose for the robot."""
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.w = 0.0  # Adjust orientation as needed
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("Initial pose set for eBot")
    
    def pose_arm(self,pose):
        """
        Purpose:
        ---
        This function controls the robot's waypoint navigation and docking process at a specific location. 
        It follows waypoints stored in `self.arm_pose`, triggers docking and payload drop actions 
        at designated waypoints, and ensures successful execution of the task.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.pose_arm()
        """
        if pose==0:
            self.navigator.followWaypoints(self.arm_pose[:])
           
        
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            global passed_point
            if feedback:
                # global current_waypoint
                current_waypoint = feedback.current_waypoint+passed_point
                # self.get_logger().info(f'Current waypoint : "{current_waypoint}"')
                # Handle actions for the first two waypoints
                if current_waypoint in [3] and not self.actions_triggered[current_waypoint-1]:
                    # self.get_logger().info(f'Initiating payload pickup at waypoint {current_waypoint}')
                    
                    docking_success = self.initiate_docking(target_distance=0.44, orientation_angle=3.19, rack_number='')  
                    if docking_success:
                        self.get_logger().info('Task Completed Successfully ')
                        time.sleep(0.8)
                        
                        self.actions_triggered[current_waypoint-1] = True 
                        passed_point =passed_point+1   
                                        
        # Check completion of first phase
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Receive  completed successfully. Proceeding to next phase.')
        else:
            self.get_logger().error(' Receive failed. Navigation halted.')
            return  # Stop further execution if phase 1 fails
        
        
        

    def conveyor2_pose(self):
        """
        Purpose:
        ---
        This function controls the robot's waypoint navigation and docking process at a specific location. 
        It follows waypoints stored in `self.conveyor2_waypoint`, triggers docking and payload drop actions 
        at designated waypoints, and ensures successful execution of the task.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.conveyor2_pose()
        """

        self.navigator.followWaypoints(self.conveyor2_waypoint[:])
        global passed_point
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # global current_waypoint
                current_waypoint = feedback.current_waypoint+passed_point
                # self.get_logger().info(f'Current waypoint : "{current_waypoint}"')
                # Handle actions for the first two waypoints
                if current_waypoint in [2] and not self.actions_triggered[current_waypoint-1]:
                    # self.get_logger().info(f'Initiating payload drop at waypoint {current_waypoint}')
                    docking_success = self.initiate_docking(target_distance=0.44, orientation_angle=2.97, rack_number='')  
                    # Proceed with payload drop once docking is successful
                    if docking_success:
                        self.get_logger().info(f'Docking successful. Initiating payload drop at waypoint {current_waypoint}')
                        time.sleep(0.8)
                        passed_point =passed_point+1
                        self.actions_triggered[current_waypoint-1] = True
                    else:
                        self.get_logger().error(f'Docking failed at waypoint {current_waypoint}. Aborting further operations.')
                        return  # Stop further execution if docking fails
                    
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('conveyor2_pose completed successfully')
        else:
            self.get_logger().error('conveyor2_pose. Navigation halted.')
            return  # Stop further execution if phase 1 fails
        


    def conveyor1_pose(self):
        """
        Purpose:
        ---
        This function controls the robot's waypoint navigation and docking process at a specific location. 
        It follows waypoints stored in `self.conveyor1_waypoint`, triggers docking and payload drop actions 
        at designated waypoints, and ensures successful execution of the task.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.conveyor1_pose()
        """
        self.navigator.followWaypoints(self.conveyor1_waypoint[:])
        global passed_point
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # global current_waypoint
                current_waypoint = feedback.current_waypoint+passed_point
                # self.get_logger().info(f'Current waypoint : "{current_waypoint}"')
                # Handle actions for the first two waypoints
                if current_waypoint in [1] and not self.actions_triggered[current_waypoint-1]:
                    # self.get_logger().info(f'Initiating payload drop at waypoint {current_waypoint}')
                    docking_success = self.initiate_docking(target_distance=0.44, orientation_angle=2.97, rack_number='')  
                    # Proceed with payload drop once docking is successful
                    if docking_success:
                        self.get_logger().info(f'Docking successful.  {current_waypoint}')
                        time.sleep(0.5)
                       
                        passed_point =passed_point+1
                        self.actions_triggered[current_waypoint-1] = True
                    else:
                        self.get_logger().error(f'Docking failed at waypoint {current_waypoint}. Aborting further operations.')
                        return  # Stop further execution if docking fails
                    
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(' conveyor1_pose completed successfully')
        else:
            self.get_logger().error('conveyor1_pose failed. Navigation halted.')
            return  # Stop further execution if phase 1 fails
        

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
        self.set_initial_pose()
        self.navigator.waitUntilNav2Active()
        global passed_point
        global boxreceived
        global total_box
        global receive_pos
        receive_pos=0
        total_box=3
        boxreceived=0
        passed_point=0
        
            
        self.get_logger().info('Going to Conveyor 1')
        self.conveyor1_pose()
        
    
        self.get_logger().info('Going to Conveyor 2')
        self.conveyor2_pose()
        
        self.get_logger().info('Going to arm Pose')
        self.pose_arm(pose=receive_pos)
       
        
        self.get_logger().info(f'Task Completed SuccessFully')
       
##################### MAIN FUNCTION #######################


def main(args=None):
    rclpy.init(args=args)
    navigation_docking_controller = NavigationDockingController()
    navigation_docking_controller.execute_navigation()
    navigation_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()