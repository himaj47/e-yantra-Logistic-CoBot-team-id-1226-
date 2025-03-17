#!/usr/bin/env python3
'''
# Team ID:          [ LB#1226 ]
# Theme:            [ Cosmo Logistic ]
# Author List:      [ Prathmesh Atkale ]
# Filename:         [ ebot_nav2_task6.py ]
# Functions:        [ create_goal_pose ,box_payload,initiate_docking,set_initial_pose, recieve_pose, conveyor_pose, box_dropping, execute_navigation, main]
# Global variables: [passed_point,  total_box, receive_pos]
'''

################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node
# from payload_service.srv import PayloadSW
from ebot_docking.srv import DockSw
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion
from tf2_ros.buffer import Buffer
import time
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from usb_servo.srv import ServoSw
from tf2_ros.transform_listener import TransformListener

from std_srvs.srv import Trigger

# import 
class NavigationDockingController(Node):
    def __init__(self):
        super().__init__('navigation_docking_controller')
        self.navigator = BasicNavigator()

        # Initialize current position
        self.current_pose = None

        # Set up odometry subscription
        
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odometry_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        
        self.receive_waypoint = [
            self.create_goal_pose(2.62, -2.80, -1.57),  # recieve pose  2.62, -2.80, 1.57
            self.create_goal_pose(2.62, -2.80, -1.57),  # recieve pose  2.62, -2.80, 1.57
        
        ]
        self.receive_waypoint1 = [
            self.create_goal_pose(2.62, -2.80, -1.57),  # recieve pose  2.62, -2.80, 1.57
            self.create_goal_pose(2.62, -2.80, -1.57),  # recieve pose  2.62, -2.80, 1.57
        
        ]
        self.receive_waypoint2 = [
            self.create_goal_pose(2.62, -2.80, -1.57),  # recieve pose  2.62, -2.80, 1.57
            self.create_goal_pose(2.62, -2.80, -1.57),  # recieve pose  2.62, -2.80, 1.57
        
        ]
        
        self.conveyor2_waypoint = [
            self.create_goal_pose(2.97, 1.84, 1.57),  # Conveyor 2  2.97, 1.84, 1.57
            self.create_goal_pose(2.97, 1.84, 1.57),  # Conveyor 2
        ]

        self.conveyor1_waypoint=[
       
            self.create_goal_pose(1.95,  -1.22, 1.57),  # Conveyor 1  1.95,  1.22, -1.57
            self.create_goal_pose(1.95,  -1.22, 1.57),  # Conveyor 1
        ]



        # Flags to ensure each action is triggered only once
        self.actions_triggered = [False, False,False,False,False,False]  # One per waypoint
        self.docking_in_progress = False  # Flag to track docking status

        self.tf_buffer=Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

        self.drop_box=self.create_client(ServoSw,'/toggle_usb_servo')
        while not self.drop_box.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Dropping of Box..')
        # Initialize docking service client

        self.imu=self.create_client(Trigger,'/reset_imu')
        while not self.imu.wait_for_service(1.0):
            self.get_logger().info(f'waiting for imu seervice ')

        self.docking_client = self.create_client(DockSw, '/dock_control')
        while not self.docking_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for DockSw service...')
        
        self.box_recieve=self.create_client(SetBool,'/passing_service')
        while not self.box_recieve.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for box_payload  service...')
            
    #Function to have Current Robot Pose and Orientation
    def odometry_callback(self, msg):
        """Update the robot's current pose."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (position.x, position.y, yaw)

   
    def box_dropping(self,pickup):
        '''
        Purpose:
        ---
        This function initiates a box-dropping action by calling a service to perform the specified servo action.  
        It sends an asynchronous service request and waits for the result.  

        Input Arguments:
        ---
        `pickup` : [bool]  
            A boolean value indicating the desired servo state for dropping the box:  
            - `True` to activate the servo for the drop action.  

        Returns:
        ---
        `success` : [bool]  
            Indicates whether the service call was successful.  

        `message` : [str]  
            A string containing additional information, typically from the service response.

        Example Usage:
        ---
        # To initiate a drop action
        success = self.box_dropping(True)
        if success:
            print('Box drop action succeeded.')
        else:
            print('Box drop action failed.')
        '''  

        req=ServoSw.Request()
        req.servostate=pickup
        future=self.drop_box.call_async(req)
        rclpy.spin_until_future_complete(self,future)
        return future.result()

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

    

    def box_payload (self,pickup):
        '''
        Purpose:
        ---
        This function initiates a payload action by calling a service to perform a specified action (e.g., dropping an item), 
         It sends an asynchronous service request and waits for the result. 

        Input Arguments:
        ---
        `pickup` : [bool]
            A boolean value indicating the desired payload action:
            - `True` for a pickup action (if implemented).
            
        Returns:
        ---
        `success` : [bool]
            Indicates whether the service call was successful.

        `message` : [str]
            A string containing additional information, typically from the service response. This may include the name of the 
            box involved in the action.

        Behavior:
        ---
        The function constructs a `SetBool.Request` object with the `pickup` value and sends it asynchronously using 
        the `box_receive` service client. It blocks execution until the service call is complete and returns the outcome 
        of the operation.

        Example Usage:
        ---
        # To initiate a drop action
        success = self.box_payload(True)
        if success:
            print('Drop action succeeded.')
        else:
            print('Drop action failed.')
        '''
        req=SetBool.Request()
        req.data=pickup
        
        future=self.box_recieve.call_async(req)
        self.get_logger().info(f"request send {req}")
        rclpy.spin_until_future_complete(self,future)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Box have put Successfully: {future.result()}')
            return future.result()
        else:
            return False
         
       
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
    
    def recieve_pose(self,pose):
        '''
        Purpose:
        ---
        This function navigates the robot to the arm pose to receive the payload. Based on the current position 
        of the robot (passed as an input), it determines the appropriate waypoints to follow and ensures the 
        robot reaches the arm pose successfully. The function also handles payload pickup actions at specific waypoints.

        Input Arguments:
        ---
        `pose` : [int]
            The current position of the robot:
                0 - Robot is at Conveyor 2.
                1 - Robot is at Conveyor 1.
                2 - Robot is at the starting point.

        Returns:
        ---
        `message` : [str]
            A message indicating the success of the docking and payload pickup operation at the arm pose.
            It returns the box name which is been received from the arm and None is returned if the navigation fails.

        Example call:
        ---
        self.recieve_pose(2)
        '''
        if pose==0:
            self.navigator.followWaypoints(self.receive_waypoint[:])
            print('from the conveyor2.........')
        elif pose==1:
            self.navigator.followWaypoints(self.receive_waypoint1[:])
            print('From the Conveyor1...........')
        elif pose==2:
            self.navigator.followWaypoints(self.receive_waypoint2[:])
            print('From starting point')

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            global passed_point
            if feedback:
                # global current_waypoint
                current_waypoint = feedback.current_waypoint+passed_point
               
                # Handle actions for the first two waypoints
                if current_waypoint in [1,3,5] and not self.actions_triggered[current_waypoint-1]:
                    if pose==2:
                        docking_success = self.initiate_docking(target_distance=0.44, orientation_angle=3.16, rack_number='')  
                    elif pose==1:
                        docking_success = self.initiate_docking(target_distance=0.44, orientation_angle=3.16, rack_number='')  
                    elif pose==0:
                        docking_success = self.initiate_docking(target_distance=0.44, orientation_angle=3.16, rack_number='')  


                    if docking_success:
                        # fut =self.imu.call_async(Trigger.Request())
                        # self.get_logger().info(f'IMU Reset {fut.result()}')
                        time.sleep(1.0)
                        box_req=self.box_payload(pickup=True) 
                        
                        if box_req.success:
                            self.get_logger().info('Receive Completed Successfully ')
                            self.actions_triggered[current_waypoint-1] = True 
                            passed_point =passed_point+1    # update passed point
                            return  box_req.message       
                   
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Receive  completed successfully. Proceeding to next phase.')
        else:
            self.get_logger().error(' Receive failed. Navigation halted.')
            return  # Stop further execution if phase 1 fails
        
        
        

    def conveyor_pose(self,box_name,conveyor):
        '''
        Purpose:
        ---
        This function navigates the robot to a specific conveyor, performs docking at the conveyor, 
        and drops the specified box onto the conveyor belt.

        Input Arguments:
        ---
        `box_name` : [str]
            The name of the box to be dropped on the conveyor belt.

        `conveyor` : [int]
            The conveyor number where the robot should navigate:
                1 - Conveyor 1.
                2 - Conveyor 2.

        Returns:
        ---
        None

        Example call:
        ---
        self.conveyor_pose(box_name='box2', conveyor=1)
        '''
        if conveyor==2:
          self.navigator.followWaypoints(self.conveyor2_waypoint[:])
        elif conveyor==1:
          self.navigator.followWaypoints(self.conveyor1_waypoint[:])

        global passed_point
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                # global current_waypoint
                current_waypoint = feedback.current_waypoint+passed_point
               
                if current_waypoint in [2,4,6] and not self.actions_triggered[current_waypoint-1]:
                  
                    if conveyor==2:
                        # docking at conveyor 2
                        docking_success = self.initiate_docking(target_distance=0.43, orientation_angle=3.16, rack_number='')  
                    elif conveyor==1:
                        # docking at conveyor 1
                        docking_success = self.initiate_docking(target_distance=0.43, orientation_angle=3.16, rack_number='')  
                            
                    # Proceed with payload drop once docking is successful
                    if docking_success:
                        # fut =self.imu.call_async(Trigger.Request())
                        # self.get_logger().info(f'IMU Reset {fut.result()}')

                        self.get_logger().info(f'Docking successful. Initiating payload drop at waypoint {current_waypoint}')
                        time.sleep(0.8)
                         
                        print(self.box_dropping(pickup=True))
                        passed_point =passed_point+1  # update passed point
                        self.actions_triggered[current_waypoint-1] = True
                    else:
                        self.get_logger().error(f'Docking failed at waypoint {current_waypoint}. Aborting further operations.')
                        return  # Stop further execution if docking fails
                    
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'conveyor{conveyor}_pose completed successfully')
        else:
            self.get_logger().error(f'conveyor{conveyor}_pose. Navigation halted.')
            return  
        

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
        # self.set_initial_pose()
        self.navigator.waitUntilNav2Active()
        global passed_point
       
        global total_box
        global receive_pos
        receive_pos=2
        total_box=3

        passed_point=0
        
        for i in range(total_box):
           
            self.get_logger().info('Going to Receive Pose')
            box=self.recieve_pose(pose=receive_pos)
            if int(box[3]) % 2 == 0:
                
                self.get_logger().info('Going to Conveyor 1')
                self.conveyor_pose(box,conveyor=1)
                receive_pos=1
            else:
                self.get_logger().info('Going to Conveyor 2')
                self.conveyor_pose(box,conveyor=2)
                receive_pos=0
        
        self.get_logger().info(f'Task Completed SuccessFully...')
       
##################### MAIN FUNCTION #######################


def main(args=None):
    rclpy.init(args=args)
    navigation_docking_controller = NavigationDockingController()
    navigation_docking_controller.execute_navigation()
    navigation_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()