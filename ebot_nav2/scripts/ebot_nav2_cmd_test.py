#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ebot_docking.srv import DockSw
from payload_service.srv import PayloadSW  # Import PayloadSW service
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math
from rclpy.duration import Duration

# Function to call the docking service using an instance of Node
def call_docking_service(node):
    client = node.create_client(DockSw, 'dock_control')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Docking service not available, waiting...')
    request = DockSw.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info('Docking service call succeeded.')
    else:
        node.get_logger().error('Docking service call failed.')

# Function to call the payload drop service
def call_payload_drop_service(node):
    payload_client = node.create_client(PayloadSW, '/payload_sw')
    while not payload_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Payload service not available, waiting...')
    req = PayloadSW.Request()
    req.receive = False  # Assuming 'receive' means picking up
    req.drop = True  # Drop payload at the drop point
    future = payload_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info('Payload drop initiated successfully.')
    else:
        node.get_logger().error('Payload drop failed.')

def main():
    rclpy.init()
    navigator = BasicNavigator()
    node = rclpy.create_node('navigation_docking_node')

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.84
    initial_pose.pose.position.y = -9.05
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.w = 0.00
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()

    drop_point = [0.43, -2.43, 3.00]
    conveyor_1 = [-4.55, 4.10, -1.00]
    conveyor_2 = [2.33, 3.93, -1.00]

    def yaw_to_quaternion(yaw):
        return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

    def create_goal_pose(x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        q = yaw_to_quaternion(yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    goal_poses = [
        create_goal_pose(drop_point[0], drop_point[1], drop_point[2]),
        create_goal_pose(conveyor_2[0], conveyor_2[1], conveyor_2[2]),
        create_goal_pose(drop_point[0], drop_point[1], drop_point[2]),
        create_goal_pose(conveyor_1[0], conveyor_1[1], conveyor_1[2])
    ]

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            node.get_logger().info(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(goal_poses)}')
            now = navigator.get_clock().now()
            if (feedback.current_waypoint == 0 or feedback.current_waypoint == 2):
             node.get_logger().info("At drop point, initiating payload drop service.")
             call_payload_drop_service(node)  # Call the drop service here at the drop point
        
            if now - nav_start > Duration(seconds=600.0):
                navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        node.get_logger().info('Mission completed successfully!')
        if feedback.current_waypoint == 0 or feedback.current_waypoint == 2:  # Assuming drop point is first & third goal
            call_payload_drop_service(node)  # Call the drop service at the drop point
    elif result == TaskResult.CANCELED:
        node.get_logger().info('Mission was canceled!')
    elif result == TaskResult.FAILED:
        node.get_logger().info('Mission failed!')
    else:
        node.get_logger().info('Unknown status of the mission!')

    call_docking_service(node)
    navigator.lifecycleShutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
