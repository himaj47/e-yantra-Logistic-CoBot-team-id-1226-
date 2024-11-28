

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
class NavigationDockingController(Node):
    def __init__(self):
        super().__init__('navigation_docking_controller')
        self.navigator = BasicNavigator()

        # Initialize current position
        self.current_pose = None

        # Set up odometry subscription
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
       
        self.phase1_waypoint = [
            self.create_goal_pose(0.58, -2.51, 1.87),  # Drop pose  0.49, -2.30, 1.87
            self.create_goal_pose(2.72, 2.88, -1.27),  # Conveyor 2  2.72, 2.88, -1.27
            self.create_goal_pose(2.72, 2.88, -1.27),  # Conveyor 2
           
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

    def odometry_callback(self, msg):
        """Update the robot's current pose."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.current_pose = (position.x, position.y, yaw)
    
    def create_goal_pose(self, x, y, yaw):
        """Create a PoseStamped goal."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
    #To calculate quaternion from the yaw
    def yaw_to_quaternion(self, yaw):
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    def initiate_payload_action(self, pickup):
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

    def initiate_docking(self, target_distance, orientation_angle, rack_number):
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
        """Run the navigation task and manage docking and payload services at specific waypoints."""
        self.set_initial_pose()
        self.navigator.waitUntilNav2Active()

        # Navigate to the first two waypoints
        self.navigator.followWaypoints(self.phase1_waypoint[:3])

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                current_waypoint = feedback.current_waypoint
                self.get_logger().info(f'Current waypoint : "{current_waypoint}"')
                # Handle actions for the first two waypoints
                if current_waypoint == 1 and not self.actions_triggered[0]:
                    self.get_logger().info('Initiating payload pickup at waypoint 1.')
                    self.initiate_payload_action(pickup=True)  # Pick up at waypoint 1
                    self.actions_triggered[0] = True
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

def main(args=None):
    rclpy.init(args=args)
    navigation_docking_controller = NavigationDockingController()
    navigation_docking_controller.execute_navigation()
    navigation_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
