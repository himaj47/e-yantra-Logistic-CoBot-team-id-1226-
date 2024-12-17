#!/usr/bin/env python3
'''
# Team ID:          [ LB#1226 ]
# Theme:            [ Cosmo Logistic ]
# Author List:      [ Prathmesh Atkale ]
# Filename:         [ nav2_latest.py ]
# Functions:        [ create_goal_pose, initiate_payload_action ,initiate_docking,set_initial_pose, execute_navigation, main]
# Global variables: [ self.current_pose ,self.phase1_waypoint,  self.phase2_waypoint, self.actions_triggered, self.docking_in_progress]
'''

################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
from payload_service.srv import PayloadSW
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
        super().__init__('navigation_docking_controller')
        self.navigator = BasicNavigator()

        # Initialize current position
        self.current_pose = None

        # Set up odometry subscription
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.phase1_waypoint = [
            self.create_goal_pose(1.16, -2.40, 3.24),  # Drop pose  0.58, -2.51, 1.87
            self.create_goal_pose( 2.42,  2.55, -1.57),  # Conveyor 2  2.72, 2.88, -1.27
            self.create_goal_pose( 2.42,  2.55, -1.57),  # Conveyor 2
           
        ]
        self.phase2_waypoint = [
            self.create_goal_pose(0.52, -2.62, -1.85),  # Drop pose
            self.create_goal_pose(-4.46, 2.89, -1.25),  #Conveyor 1 -4.46, 2.89, -1.25
            self.create_goal_pose(-4.46, 2.89, -1.25)   # Conveyor 1  
        ]

        # Flags to ensure each action is triggered only once
        self.actions_triggered = [False, False, False, False]  # One per waypoint
        self.docking_in_progress = False  # Flag to track docking status
        # Initialize payload service client
        self.payload_client = self.create_client(PayloadSW, '/payload_sw')
        while not self.payload_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for PayloadSW service...')

        # Initialize docking service client
        self.docking_client = self.create_client(DockSw, '/dock_control')
        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for DockSw service...')
        
        self.box_recieve=self.create_client(SetBool,'/passing_service')
        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for box_payload  service...')
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

    def initiate_payload_action(self, pickup):
        '''
        Purpose:
        ---
        This function initiates a payload action by calling a service to  drop an item, 
        based on the input argument. It returns the result of the service call, including success status 
        and a message containing the name of the box involved.

        Input Arguments:
        ---
        `pickup` : [bool]
            A boolean value indicating the desired payload action:
            - `False` to perform a drop action.

        Returns:
        ---
        `success` : [bool]
            Indicates whether the service call was successful.
            
        `message` : [str]
            A string containing additional information, typically the name of the box involved 
            in the payload action.

        Example call:
        ---
        # To initiate a pickup action
        success, message = self.initiate_payload_action(False)
        if success:
            print(f'Action succeeded: {message}')
        else:
            print('Action failed')
        '''


        """Call the payload service to either pick up or drop."""
        req = PayloadSW.Request()
        req.receive = pickup
        req.drop = not pickup
        future = self.payload_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            action = "Pickup" if pickup else "Drop"
            self.get_logger().info(f'{action} action succeeded with message: {future.result().message}')
        else:
            self.get_logger().error('Payload service call failed.')
    

    def box_payload (self,pickup):
        req=SetBool.Request()
        req.data=pickup
        self.navigator.cancelTask()
        
        future=self.box_recieve.call_async(req)
        self.get_logger().info(f"request send {req}")
        rclpy.spin_until_future_complete(self,future)
        # rate = self.create_rate(2, self.get_clock())
        
        # if future.result() is not None and future.result().success:
        #     self.get_logger().info(f"Box successful with message: {future.result().message}")
        #     return True
        # else:
        #     self.get_logger().error('Box service call failed')
        #     return False
        response=SetBool.Response()
        # self.vel=Twist()
        # while not response.success:
        #     self.get_logger().info("Waiting for the box to be placed ...")
        #     future=self.box_recieve.call_async(req)
        #     rclpy.spin_until_future_complete(self,future)
            # self.navigator.cancelTask()
            # self.vel=Twist()
            # self.vel.linear.x=0.0
            # self.vel.angular.z=0.0
            # self.cmd_vel_pub.publish(self.vel)
            # rate.sleep()
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Box have put Successfully: {future.result()}')
            return True
        else:
            # self.navigator.cancelTask()
            return False
            # rate.sleep()
        # if response.success:
        #      self.get_logger().info(f'Box have put Successfully: {future.result()}')
        #      return True
        # else:
        #     self.get_logger().info(f'Box Service failed')

        #     return False
        
   
       
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
        initial_pose.pose.position.x = 1.84
        initial_pose.pose.position.y = -9.05
        initial_pose.pose.position.z = 0.0
        initial_pose.pose.orientation.w = 0.0  # Adjust orientation as needed
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("Initial pose set for eBot")
        
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

        # Navigate to the first two waypoints
        self.navigator.followWaypoints(self.phase1_waypoint[:])

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                global current_waypoint
                current_waypoint = feedback.current_waypoint
                self.get_logger().info(f'Current waypoint : "{current_waypoint}"')
                # Handle actions for the first two waypoints
                if current_waypoint == 1 and not self.actions_triggered[0]:
                    self.get_logger().info('Initiating payload pickup at waypoint 1.')
                    
                   
                    box_req=self.box_payload(pickup=True)  # Pick up at waypoint 1
                    
                    if box_req:
                        self.get_logger().info('Task Completed Successfully ')
                        self.actions_triggered[0] = True
                    
                        self.navigator.followWaypoints(self.phase1_waypoint[1:])
                        self.feed = self.navigator.getFeedback()
                        current_waypoint = self.feed.current_waypoint+1
                        self.get_logger().info(f'Current feeddback is :"{ current_waypoint}"')
                    # self.current_waypoint = feedback.current_waypoint+1
                elif current_waypoint == 2 and not self.actions_triggered[1]:      
                    docking_success = self.initiate_docking(target_distance=0.08, orientation_angle=1.57, rack_number='')              
                    # Proceed with payload drop once docking is successful
                    if docking_success:
                        self.get_logger().info('Docking successful. Initiating payload drop at waypoint 2.')
                        self.initiate_payload_action(pickup=False)  # Drop at waypoint 2
                        self.actions_triggered[1] = True
                    else:
                        self.get_logger().error('Docking failed at waypoint 2. Aborting further operations.')
                        return  # Stop further execution if docking fails
                    self.actions_triggered[1] = True
        # Check completion of first phase
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Phase 1 completed successfully. Proceeding to next phase.')
        else:
            self.get_logger().error('Phase 1 failed. Navigation halted.')
            return  # Stop further execution if phase 1 fails

        # Reset the navigator for next phase navigation
        self.navigator.followWaypoints(self.phase2_waypoint[:])

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                current_waypoint = feedback.current_waypoint+2  # +2 for  completion of previous  two waypoint 

                # Handle actions for the last two waypoints
                if current_waypoint == 3 and not self.actions_triggered[2]:
                    self.get_logger().info('Initiating payload pickup at waypoint 3.')
                    self.initiate_payload_action(pickup=True)  # Pick up at waypoint 3
                    self.actions_triggered[2] = True
                elif current_waypoint == 4 and not self.actions_triggered[3]:
                    docking_success = self.initiate_docking(target_distance=0.08, orientation_angle=1.57, rack_number='')                 
                    # Proceed with payload drop once docking is successful
                    if docking_success:
                        self.get_logger().info('Docking successful. Initiating payload drop at waypoint 4.')
                        self.initiate_payload_action(pickup=False)  # Drop at waypoint 4
                        self.actions_triggered[3] = True
                    else:
                        self.get_logger().error('Docking failed at waypoint 4. Aborting further operations.')
                        return  # Stop further execution if docking fails
                    self.actions_triggered[3] = True
        # Check final result
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Mission completed successfully!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Mission was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Mission failed!')


##################### MAIN FUNCTION #######################


def main(args=None):
    rclpy.init(args=args)
    navigation_docking_controller = NavigationDockingController()
    navigation_docking_controller.execute_navigation()
    navigation_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
