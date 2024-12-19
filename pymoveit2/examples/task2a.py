#!/usr/bin/env python3

"""
to-do:
- class to store for eg. all states (not sure rn)
- correction in task 1b's code (change orientation of transforms and transforms not publishing - issues)
- take the code for servoing from examples of pymoveit package
- Implement lookup transforms from end effector to base_link & boxes (x, y, z and orientations)
- set max velocities
- Implement PID control for x, y, z and orientation of the end effector (set points from above)
- each component (x, y, ...) should have its weights -> Kp, Kd & Ki
- marker feedback for x, y, z and orientation (for PID control)
- servoing the end effector with velocities calculated using PID control function
- create client for /SERVOLINK service (To remove the ArUco box from the Ebot) (see the instructions section)

"""

"""
algorithm:
- orientation (EEF face down) 
- position (box)
- attach service 
- default position and orientation
- drop location
- release box and go to next box
- after collecting all boxes -> default position and orientation

"""


import math, time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import rclpy.time
from rclpy.clock import Clock

from pymoveit2.robots import ur5
import tf_transformations

from pymoveit2 import MoveIt2Servo
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from servo_msgs.srv import ServoLink

from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage

import yaml
from functools import partial

EEF_target_frame = "wrist_3_link"   # EEF frame
box_target_frame = "obj_"           # box frame
source_frame = "base_link"

# max velocities
max_lin_vel = 3.0
max_ang_vel = 3.0

prev_error = 0.0

# EEF pose & quat
EEF_link = {
    "position": [0.0129, 0.5498, 0.1333],
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094],
    "euler_angles": [0, 0, 0]
}

# state of shoulder pan joint
shoulder_pan_joint = 0.0     # in randians (r p y)

# left box pose
lBoxPose = {
    "box_name": "box2",
    "position": ["lBoxPose", 0.0, 0.0, 0.0],    # ["lBoxPose", 0.06086, 0.54941, 0.12435]
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094]
}

# right box pose 
rBoxPose = {
    "box_name": "box3",
    "position": ["rBoxPose", 0.0, 0.0, 0.0],  # ["rBoxPose", 0.0227, -0.5212, 0.1383]     ["rBoxPose", 0.1139, -0.5212, 0.5539]    ["rBoxPose", 0.03092, -0.52645, 0.12331]
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094]
}

# ur5 configs
ur5_configs = {

    "default_config": {
        "position": ["default_config", 0.1627, 0.1081, 0.4664],
        "quaternion": [0.5, 0.5, 0.5, 0.5]
    },

    "start_config": {
        "position": ["start_config", 0.4652, 0.1002, 0.4579],
        "quaternion": [0.71035, 0.70383, 0.00303, -0.00292],
        "euler_angles": tf_transformations.euler_from_quaternion([0.71035, 0.70383, 0.00303, -0.00292])
    },

    "drop_config": {
        "position": ["drop_config", 0.4835, -0.0030, -0.0380],
        "quaternion": [0.71035, 0.70383, 0.00303, -0.00292]
    },

    "rbTopPose": {
        "position": ["rbTopPose", 0.1117, -0.4626, 0.4579],   #0.1627, -0.3386, 0.7063
        "quaternion": [0.71035, 0.70383, 0.00303, -0.00292]
    },
    "lbTopPose": {
        "position": ["lbTopPose", -0.1077, 0.4635, 0.4579],
        "quaternion": [0.71035, 0.70383, 0.00303, -0.00292]
    },

}

# execution flag
execute = True

# signal flag
signal = True  # True
aruco_transforms = []

# box attached to EEF
box_attached = ""

# order of tasks
task_queue = []
task_ptr = 0

# scheduling task
def schedule_tasks(box_pose=None, end=False):
    # for box_pos in box_poses:
    if box_pose != None:
        task_queue.append(ur5_configs["start_config"]["position"])

        if box_pose[2] > 0:
            task_queue.append(ur5_configs["lbTopPose"]["position"])
            task_queue.append(box_pose)
            task_queue.append(ur5_configs["lbTopPose"]["position"])

        else:
            task_queue.append(ur5_configs["rbTopPose"]["position"])
            task_queue.append(box_pose)
            task_queue.append(ur5_configs["rbTopPose"]["position"])

        task_queue.append(ur5_configs["start_config"]["position"])
        task_queue.append(ur5_configs["drop_config"]["position"])
    
    if end:
        task_queue.append(ur5_configs["start_config"]["position"])
        task_queue.append(ur5_configs["default_config"]["position"])

# callbacks
def mag_on_callback(future):
    try:
        response = future.result()
        # print(f"response from /GripperON: {response}")
    except Exception as e:
        print("error: {e}")

def mag_off_callback(future):
    try:
        response = future.result()
        # print(f"response from /GripperOFF: {response}")
    except Exception as e:
        print("error: {e}")

def rm_box_callback(future):
    try:
        response = future.result()
        # print(f"response from /SERVOLINK: {response}")
    except Exception as e:
        print("error: {e}")

# clients
def magnet_on(box_name: str | None, gripper_control_attach):
    req = AttachLink.Request()
    req.model1_name =  box_name     
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    future = gripper_control_attach.call_async(req)
    future.add_done_callback(mag_on_callback)
    # print(f"request for magnet on for box {box_name} is sent!!")

def magnet_off(box_name: str | None, gripper_control_detach):
    req = DetachLink.Request()
    req.model1_name =  box_name     
    req.link1_name  = 'link'       
    req.model2_name =  'ur5'       
    req.link2_name  = 'wrist_3_link'  

    future = gripper_control_detach.call_async(req)
    future.add_done_callback(mag_off_callback)
    # print(f"request for magnet off for box {box_name} is sent!!")

def remove_box(box_name: str | None, servo_control):
    req = ServoLink.Request()
    req.box_name =  box_name    
    req.box_link  = 'link'       
    future = servo_control.call_async(req)
    future.add_done_callback(rm_box_callback)

# PID control
def PID_controller(error, Kp, Kd = 0, Ki = 0):
    prev_error = 0.0
    vel = (Kp * error) + (Kd * (error-prev_error)) + (Ki * 1)
    vel = max(min(vel, max_lin_vel), -max_lin_vel)  # change this for angular too
    prev_error = error
    return round(vel, 4)

# checking if goal reached (with tolerance)
def goal_reached(error, tolerance = 0.005):
    if abs(error) <= tolerance:
        return True
    
    return False

# check if task done or not for a box number (0 - not done && 1 - done)
task_done = [0]*15

# if the flag is False then EEF goes back to default/resting position if no aruco box detected
flag = True

# get aruco frame names
def get_all_frames(tf_buffer):
    frames_yaml = tf_buffer.all_frames_as_yaml()
    frames_dict = yaml.safe_load(frames_yaml)

    for frame in frames_dict:
        if frame.startswith("obj_"):
            if frame != "obj_12":
                frame_id = int(frame.strip("obj_"))

                if not task_done[frame_id]:
                    aruco_transforms.append(frame)
                    

# ur5 controller
def servo_motion(moveit2_servo, gripper_control_attach, gripper_control_detach, servo_control):
    global execute
    global task_ptr
    global box_attached
    global signal

    if len(task_queue):
        if execute:
            # PID control for EEF orientation
            error_ang_x = ur5_configs["start_config"]["euler_angles"][0] - EEF_link["euler_angles"][0]
            ang_vel_Y = PID_controller(error=error_ang_x, Kp=3.8)
            
            moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, ang_vel_Y, 0.0))

            if goal_reached(error_ang_x):
                execute = False
            
        else:
            # PID control for EEF position
            error_x = task_queue[task_ptr][1] - EEF_link["position"][0]
            error_y = task_queue[task_ptr][2] - EEF_link["position"][1]
            error_z = task_queue[task_ptr][3] - EEF_link["position"][2]

            # print("error_x: " + str(error_x), "  error_y: " + str(error_y), "  error_z: " + str(error_z))

            if (goal_reached(error_x) and goal_reached(error_y) and goal_reached(error_z)):
                if task_queue[task_ptr][0] == "lBoxPose":
                    box_attached = lBoxPose["box_name"]
                    magnet_on(lBoxPose["box_name"], gripper_control_attach)

                elif task_queue[task_ptr][0] == "rBoxPose":
                    box_attached = rBoxPose["box_name"]
                    magnet_on(rBoxPose["box_name"], gripper_control_attach)

                elif task_queue[task_ptr][0] == "drop_config":
                    print("box attached: " + str(box_attached))
                    magnet_off(box_name=box_attached, gripper_control_detach=gripper_control_detach)
                    remove_box(box_attached, servo_control)

                    if len(aruco_transforms) == 0:
                        signal = True
                    else:
                        aruco_transforms.pop(0)

                if task_ptr < len(task_queue)-1: task_ptr += 1

            else:
                ln_vel_X = PID_controller(error=error_x, Kp=3.8)
                ln_vel_Y = PID_controller(error=error_y, Kp=3.8)
                ln_vel_Z = PID_controller(error=error_z, Kp=3.8)

                # print("ln_vel_X: " + str(ln_vel_X), "  ln_vel_Y: " + str(ln_vel_Y), "  ln_vel_Z: " + str(ln_vel_Z))
                
                moveit2_servo(linear=(ln_vel_X, ln_vel_Y, ln_vel_Z), angular=(0.0, 0.0, 0.0))


# checking all necessary transforms
def check_transform(tf_buffer, node):
    global aruco_transforms
    global signal
    global flag

    try:
        # feedback
        EEF_to_base = tf_buffer.lookup_transform(     # EEF w.r.t. base_link
            source_frame,
            EEF_target_frame,
            rclpy.time.Time()
        )

        # base_link to box transforms (aruco transforms)
        if signal:
            get_all_frames(tf_buffer=tf_buffer)

            if len(aruco_transforms) > 0:
                flag = False
                signal = False

                for tranform in aruco_transforms:
                    box_num = int(tranform.strip("obj_"))

                    if not task_done[box_num]:
                        base_to_box = tf_buffer.lookup_transform(
                                                "base_link",
                                                tranform,
                                                rclpy.time.Time())
                    
                        if base_to_box.transform.translation.y > 0:
                            lBoxPose["position"][0] = "lBoxPose" 
                            lBoxPose["position"][1] = base_to_box.transform.translation.x
                            lBoxPose["position"][2] = base_to_box.transform.translation.y
                            lBoxPose["position"][3] = base_to_box.transform.translation.z
                            lBoxPose["box_name"] = "box" + tranform.strip("obj_")
                            schedule_tasks(box_pose=lBoxPose["position"])
                        else:
                            rBoxPose["position"][0] = "rBoxPose" 
                            rBoxPose["position"][1] = base_to_box.transform.translation.x 
                            rBoxPose["position"][2] = base_to_box.transform.translation.y
                            rBoxPose["position"][3] = base_to_box.transform.translation.z
                            rBoxPose["box_name"] = "box" + tranform.strip("obj_")
                            schedule_tasks(box_pose=rBoxPose["position"])

                        task_done[box_num] = 1

            elif not flag:
                schedule_tasks(end=True)
                flag = True
        
        # node.get_logger().info(str(EEF_to_base.transform.translation))
        EEF_link["position"] = [EEF_to_base.transform.translation.x, EEF_to_base.transform.translation.y, EEF_to_base.transform.translation.z]
        EEF_link["quaternion"] = [EEF_to_base.transform.rotation.x, EEF_to_base.transform.rotation.y, EEF_to_base.transform.rotation.z, EEF_to_base.transform.rotation.w]
        EEF_link["euler_angles"] = tf_transformations.euler_from_quaternion(EEF_link["quaternion"])
        EEF_to_base.transform

    except TransformException as ex:
        node.get_logger().info(
            f'Could not transform {EEF_target_frame} to {source_frame}: {ex}')
        

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 Servo interface
    moveit2_servo = MoveIt2Servo(
        node=node,
        frame_id=ur5.base_link_name(),
        callback_group=callback_group,
        enable_at_init=True
    )

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    # clients 
    gripper_control_attach = node.create_client(AttachLink, '/GripperMagnetON')
    gripper_control_detach = node.create_client(DetachLink, '/GripperMagnetOFF')
    servo_control = node.create_client(ServoLink, '/SERVOLINK')

    while not gripper_control_attach.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('EEF /GripperMagnetON service not available, waiting again...')

    while not gripper_control_detach.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('EEF service /GripperMagnetOFF not available, waiting again...')

    while not servo_control.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Servo service not available, waiting again...')


    # timer for servo motion
    servoing = node.create_timer(0.03, partial(servo_motion, moveit2_servo, gripper_control_attach, gripper_control_detach, servo_control))

    # timer for position and orientation feedback
    transform_checker = node.create_timer(0.03, partial(check_transform, tf_buffer, node))

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()