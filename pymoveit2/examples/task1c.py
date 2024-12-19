#!/usr/bin/env python3
"""
Example of moving to a joint configuration.
`ros2 run pymoveit2 ex_joint_goal.py --ros-args -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57]"`
"""

# Task 1C
# P1 → D → P2 → D → P3 → D

# P1:   [ 0.20, -0.47, 0.65 ]
# P2:   [ 0.75,0.49,-0.05 ]
# P3:   [ 0.75,-0.23,-0.05 ]
# D:    [ -0.69, 0.10, 0.44 ]


"""
write down all the task's pos, qua_xyz, joint states:

default joint states = [0.0, -2.39, 2.4, -3.15, -1.58, 3.15]
default_position = [0.15, 0.106, 0.434]
default_quaXYZ = [0.50, 0.50, 0.50, 0.50]

[
shoulder_pan_joint: 0.0,
shoulder_lift_joint: -2.39,
elbow_joint: 2.4,
wrist_1_joint: -3.15,
wrist_2_joint: -1.58,
wrist_3_joint: 3.15
]

initial_joint_states: [-0.03440792241516388, -1.8525370534617431, 1.626401766838843, -2.915457367312677, -1.536388404379315, 3.141592656758572]


order of tasks:
initially at the default state

pickup_right/left_box
1. go to initial joint state
2. pickup front right/left box
3. go back to the default_state
4. place the box (keep some offset for left and right box)

pickup_rack_box
1. go to the box location
2. move back a little and then down
3. back to the default state 
4. place the box (give offset)
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from pymoveit2 import MoveIt2Servo

from linkattacher_msgs.srv import AttachLink, DetachLink
import math


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    # create client to turn on/off gripper magnet
    gripper_control_mag_on = node.create_client(AttachLink, '/GripperMagnetON')
    gripper_control_mag_off = node.create_client(DetachLink, '/GripperMagnetOFF')

    while not gripper_control_mag_on.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('EEF /GripperMagnetON service not available, waiting again...')  

    while not gripper_control_mag_off.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('EEF service /GripperMagnetOFF not available, waiting again...')   

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    moveit2.max_cartesian_speed = 10.0
    moveit2.max_velocity = 10.0

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
    

    # # positions
    # rack_box_p = [0.20, -0.47, 0.65]     # box on the rack
    # rack_box_q = [0.70, 0.0, 0.0, 0.70]

    # up_p = [0.20, -0.47, 0.69]
    # up_q = [0.70, 0.0, 0.0, 0.70]

    # back_p = [0.20, -0.23, 0.69]
    # back_q = [0.70, 0.0, 0.0, 0.70]

    # # for boxes in front of the arm
    # l_box_p = [0.75, 0.40, -0.05]        # front left box
    # l_box_q = [0.70, 0.70, 0.0, 0.0]

    # r_box_p = [0.75, -0.23, -0.05]       # front right box/GripperMagnetON
    # r_box_q = [0.70, 0.70, 0.0, 0.0]   

    # # drop location
    # D_p = [-0.69, 0.10, 0.44]            # place the box
    # D_q = [0.50, -0.50, -0.50, 0.50]

    # intermediate_D_p = [-0.017, 0.106, 0.975]
    # intermediate_D_q = [0.50, 0.50, 0.50, 0.50]

    # # for boxes in front
    # initial_pos = [0.35, 0.10, 0.68]
    # initial_qua = [0.70, 0.70, 0.0, 0.0]

    # # default location
    # default_pos = [0.15, 0.106, 0.434]
    # default_qua = [0.50, 0.50, 0.50, 0.50]


    # configurations
    default_config = [0.0, -2.39, 2.4, -3.15, -1.58, 3.15]
    # intermediate_drop_config = [0.03228390087724188, -1.9056287337336115, 0.446813575948141, -1.6827774961378348, -1.6030802276665423, 3.1415926487355765]
    # drop_config = [0.015076042028528523, -2.122867550244782, -0.8038727249499437, -3.3564450453425194, -1.55572028731518, -4.215019933905765]
    front_left_box_config = [math.radians(21.0), math.radians(-17.0), math.radians(44.0), math.radians(-117.0), math.radians(-89.0), math.radians(197.0)]
    front_right_box_config = [math.radians(-26.0), math.radians(-28.0), math.radians(67.0), math.radians(-130.0), math.radians(-89.0), math.radians(155.0)]
    rack_box_config = [math.radians(-77.0), math.radians(-86.0), math.radians(78.0), math.radians(-172.0), math.radians(-101.0), math.radians(271.0)]
    # up_config = [math.radians(-76.0), math.radians(-85.0), math.radians(72.0), math.radians(-167.0), math.radians(-101.0), math.radians(271.0)]
    back_config = [math.radians(-55.0), math.radians(-112.0), math.radians(93.0), math.radians(-161.0), math.radians(-125.0), math.radians(181.0)]

    # drop configurations
    left_box_dl = [math.radians(0.0), math.radians(-106.0), math.radians(-92.0), math.radians(-162.0), math.radians(-91.0), math.radians(180.0)]
    right_box_dl = [math.radians(0.0), math.radians(-137.0), math.radians(-34.0), math.radians(-190.0), math.radians(-90.0), math.radians(181.0)]
    rack_box_dl = [math.radians(38.0), math.radians(-115.0), math.radians(-87.0), math.radians(-159.0), math.radians(-52.0), math.radians(182.0)]

    def mag_on_callback(future):
        try:
            response = future.result()
            print(f"response from /GripperON: {response}")
        except Exception as e:
            print("error: {e}")

    def mag_off_callback(future):
        try:
            response = future.result()
            print(f"response from /GripperOFF: {response}")
        except Exception as e:
            print("error: {e}")


    def magnet_on(box_name: str | None):
        req = AttachLink.Request()
        req.model1_name =  box_name     
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        future = gripper_control_mag_on.call_async(req)
        future.add_done_callback(mag_on_callback)
        print(f"request for magnet on for box {box_name} is sent!!")

    def magnet_off(box_name: str | None):
        req = DetachLink.Request()
        req.model1_name =  box_name     
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        future = gripper_control_mag_off.call_async(req)
        future.add_done_callback(mag_off_callback)
        print(f"request for magnet off for box {box_name} is sent!!")

    # tasks
    def pickup_box(box):
        positions = []
        joints_poses = []
        box_name = ""
        if box == "right":
            box_name = "box49"
            joints_poses.append(front_right_box_config)
            go_through_poses(joint_state=joints_poses)
            magnet_on(box_name)

        elif box == "left":
            box_name = "box3"
            joints_poses.append(front_left_box_config)
            go_through_poses(joint_state=joints_poses)
            magnet_on(box_name)

        elif box == "rack":
            box_name = "box1"
            joints_poses.append(rack_box_config)

            go_through_poses(joint_state=joints_poses)
            magnet_on(box_name)

            positions.clear()
            joints_poses.clear()

            joints_poses.append(back_config)
            go_through_poses(joint_state=joints_poses)

        positions.clear()
        joints_poses.clear()

        if box == "right":
            joints_poses.append(right_box_dl)

        elif box == "left":
            joints_poses.append(left_box_dl)

        elif box == "rack":
            joints_poses.append(rack_box_dl)

        go_through_poses(joint_state=joints_poses)
        positions.clear()
        joints_poses.clear()
        magnet_off(box_name)

        if box == "rack":
            joints_poses.append(default_config)

        go_through_poses(positions, joints_poses)
        positions.clear()
        joints_poses.clear()

    def go_through_poses(positions_n_quaternion = None, joint_state = None):
        trajectory = []
        current_joint_state = [0.0, -2.39, 2.4, -3.15, -1.58, 3.15]

        # calculate joint positions and append to trajectory
        if positions_n_quaternion != None:
            for pos, quat_xyzw in positions_n_quaternion:
                print("position: " + str(pos) + "   quaternion: " + str(quat_xyzw))
                joint_states = moveit2.compute_ik(position=pos, quat_xyzw=quat_xyzw, start_joint_state=current_joint_state)
                if joint_states != None:
                    trajectory.append(joint_states.position.tolist())
                if len(current_joint_state) != 0:
                    current_joint_state = trajectory[-1]  # last computed joint states
                print(f"current joint states: {current_joint_state}")

            # call the move_to_configuration and wait_till_executed
            for joint_pose in trajectory:
                node.get_logger().info(f"Moving to {{joint_positions: {joint_pose}}}")
                moveit2.move_to_configuration(joint_pose, cartesian=True, tolerance=0.009)
                moveit2.wait_until_executed()
            
            trajectory.clear()

        if joint_state != None:
            for joint_angles in joint_state:
                trajectory.append(joint_angles)

            for joint_pose in trajectory:
                node.get_logger().info(f"Moving to {{joint_positions: {joint_pose}}}")
                moveit2.move_to_configuration(joint_pose, cartesian=True)
                moveit2.wait_until_executed()
        
        else:
            print("no pose given!!")

    # executing tasks
    pickup_box("left")
    pickup_box("right")
    pickup_box("rack")

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()