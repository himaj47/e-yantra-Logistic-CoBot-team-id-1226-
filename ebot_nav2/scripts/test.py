#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Int32
import cmd
import math, statistics
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import yaml
from scipy.spatial.transform import Rotation as R
from ebot_docking.srv import DockSw 
from functools import partial
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

with open('/home/eyantra/LB24/Student/1526/install/ebot_nav2/lib/ebot_nav2/config.yaml', 'r') as read_file:
    contents = yaml.safe_load(read_file)


drop_pose_xy_yaw = contents['pre_dock_position'][2]['arm_pose']
conveyor_2_xy_yaw = contents['pre_dock_position'][1]['conveyor_2']
conveyor_1_xy_yaw = contents['pre_dock_position'][0]['conveyor_1']

initial_pose1 = [1.83889,-9.050318,0.054997,3.139789]
#linear_vel = 0.3
angular_vel = 0.0
print("drop pose xy, yaw:", drop_pose_xy_yaw)
print("conveyor 1 xy, yaw:", conveyor_1_xy_yaw)
print("conveyor 2 xy, yaw:", conveyor_2_xy_yaw)
r1 = R.from_euler('xyz', [0, 0,drop_pose_xy_yaw[2]], degrees=False)
r2 = R.from_euler('xyz', [0, 0,conveyor_1_xy_yaw[2]], degrees=False)
r3 = R.from_euler('xyz', [0, 0,conveyor_2_xy_yaw[2]], degrees=False)
r1=r1.as_quat()
r2=r2.as_quat()
r3=r3.as_quat()


class PayLoad(Node):
    def __init__(self):
        super().__init__('payload_client')
    
    def run_payload(self,receive,drop):
        box_payload = self.create_client(PayloadSW, '/payload_sw')

        while not box_payload.wait_for_service(1.0):
            self.get_logger().warn('Payload service not available')

        request = PayloadSW.Request()
        request.receive = receive
        request.drop = drop
        future = box_payload.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(response.message + " dropped")
        except Exception as e:
            self.get_logger().error("service call failed %r" %(e, ))

class Docking(Node):
    def __init__(self):
        super().__init__('Docking_client')

    def run_docker(self,distance1,distance2,orientation,rack_no,undocking):
        ebot_docker = self.create_client(DockSw, '/dock_control')

        while not ebot_docker.wait_for_service(1.0):
            self.get_logger().warn('Docking service not available')

        request = DockSw.Request()
        request.distance1 = distance1
        request.distance2 = distance2
        request.orientation = orientation
        request.rack_no = str(rack_no)
        request.undocking = undocking
        future = ebot_docker.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("service call failed %r" %(e, ))

class GlobalRadius(Node):
    def __init__(self):
        super().__init__('param_change')
    def param_change(self, value1, value2):
        changer = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        while not changer.wait_for_service(1.0):
            self.get_logger().warn('Service not available')
        request = SetParameters.Request()
        param1 = Parameter(name='robot_radius', value=value1)   #type=Parameter.Type.NOT_SET,
        param2 = Parameter(name='inflation_layer.inflation_radius', value=value2)

        request.parameters = [param1.to_parameter_msg()]
        request.parameters = [param2.to_parameter_msg()]
        future = changer.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info('robot radius changed to %s' % value1)
            self.get_logger().info('global inf radius changed to %s' % value2)
        except Exception as e:  
            self.get_logger().error("Service call failed %r" % (e,))

class LocalRadius(Node):
    def __init__(self):
        super().__init__('local_costmap')
    def local_costmap(self, value):
        changer = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        while not changer.wait_for_service(1.0):
            self.get_logger().warn('Service not available')
        request = SetParameters.Request()
        param = Parameter(name='inflation_layer.inflation_radius', value=value)
        request.parameters = [param.to_parameter_msg()]
        future = changer.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info('local inf radiue changed to %s' % value)
        except Exception as e:  
            self.get_logger().error("Service call failed %r" % (e,))




class MyBotNavigator(Node):
    def __init__(self):
        super().__init__('my_bot_navigator')
        self.velocity_sub= self.create_subscription(
        Twist,
        '/cmd_vel',
        self.cmd_vel_callback,
        qos_profile=10
        )
        
        self.linear_vel = [0.0, 0.0, 0.0]
        
        self.navigator = BasicNavigator()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'odom'  #'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = initial_pose1[0]   
        initial_pose.pose.position.y = initial_pose1[1]  
        initial_pose.pose.orientation.z = initial_pose1[2]  
        initial_pose.pose.orientation.w =initial_pose1[3]  
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()

        
        ##Go to first pre undock pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = conveyor_1_xy_yaw[0] -0.5
        goal_pose.pose.position.y = conveyor_1_xy_yaw[1] -0.05
        goal_pose.pose.orientation.z =  r3[2]
        goal_pose.pose.orientation.w = r3[3]

       

        self.navigator.goToPose(goal_pose)
        
        i = 0
        while not self.navigator.isTaskComplete():
        # ################################################
        # #
        # # Implement some code here for your application!
        # #
        # ################################################
            # self.publisher_function()
        # # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                eta_in_seconds = self.navigator.get_clock().now().to_msg().sec - goal_pose.header.stamp.sec
                print(f'time taken: {eta_in_seconds:.0f} seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    
                    self.navigator.goToPose(goal_pose)


                # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Conveyor 2 reached!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        ## Call the docking service
        docker = Docking()
        docker.run_docker(conveyor_1_xy_yaw[0],conveyor_1_xy_yaw[1]-0.13,conveyor_1_xy_yaw[2]+0.08, 'conveyor_1', True)


        # Go to conveyor belt 1
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = conveyor_2_xy_yaw[0] - 0.35  #0.11
        goal_pose.pose.position.y = conveyor_2_xy_yaw[1] 
        goal_pose.pose.orientation.z =  r2[2]
        goal_pose.pose.orientation.w = r2[3]

       

        self.navigator.goToPose(goal_pose)
        
        i = 0
        while not self.navigator.isTaskComplete():
        # ################################################
        # #
        # # Implement some code here for your application!
        # #
        # ################################################
            # self.publisher_function()
        # # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                eta_in_seconds = self.navigator.get_clock().now().to_msg().sec - goal_pose.header.stamp.sec
                print(f'time taken: {eta_in_seconds:.0f} seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    
                    self.navigator.goToPose(goal_pose)


                # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Conveyor 1 reached!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        ## Call the docking service
        #time.sleep(2.0)
        docker.run_docker(conveyor_2_xy_yaw[0],conveyor_2_xy_yaw[1],conveyor_2_xy_yaw[2], 'conveyor_2', True)

        # Go to drop pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = drop_pose_xy_yaw[0]
        goal_pose.pose.position.y = drop_pose_xy_yaw[1]-0.3
        goal_pose.pose.orientation.z = r1[2]
        goal_pose.pose.orientation.w = r1[3]
        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isTaskComplete():
        #     ################################################
        #     #
        #     # Implement some code here for your application!
        #     #
        #     ################################################
        #     # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                eta_in_seconds = self.navigator.get_clock().now().to_msg().sec - goal_pose.header.stamp.sec
                print(f'time taken: {eta_in_seconds:.0f} seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                
                    self.navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Drop pose reached!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        
        docker.run_docker(drop_pose_xy_yaw[0],drop_pose_xy_yaw[1],drop_pose_xy_yaw[2]+0.1, 'drop_pose', True)

        self.navigator.lifecycleShutdown()

    def cmd_vel_callback(self,cmd):
        print("Callback triggered with cmd_vel:", cmd)
        self.linear_vel = cmd.linear
        self.angular_vel = cmd.angular
        print(f"Received velocity: linear={cmd.linear.x}, angular={cmd.angular.z}")




    # Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)
    node= Node("task_1A")
    
    def odometry_callback(msg):
        global initial_pose1
        # Extract and update robot pose information from odometry message
        initial_pose1[0] = msg.pose.pose.position.x
        initial_pose1[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        initial_pose1[2] = quaternion_array.z
        initial_pose1[3] = quaternion_array.w
        node.destroy_subscription(odom_sub)
    odom_sub=node.create_subscription(
            Odometry,
            'odom',
            odometry_callback,
            10
            )

    rclpy.spin_once(node)
    global initial_pose1
    print(initial_pose1)
    
    my_bot_navigator = MyBotNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(my_bot_navigator)

    executor.spin()

    my_bot_navigator.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()