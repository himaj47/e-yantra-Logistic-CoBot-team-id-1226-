#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
# Team ID:          1226
# Theme:            Logistic coBot
# Author List:      Himaj Joshi
# Filename:         passing.py
# Functions:        __init__, handle_request, check_transform, schedule_tasks, get_all_frames, mag_on_callback, mag_off_callback, rm_box_callback, magnet_on, magnet_off, PID_controller, goal_reached, servo_motion, main
# Global variables: signal, aruco_transforms, box_aruco_frame, flag, task_queue, task_ptr, srv, placed, fstat, netWrench, lBoxPose, rBoxPose, EEF_link, ur5_configs
'''

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import rclpy.time

from pymoveit2.robots import ur5
import tf_transformations

from pymoveit2 import MoveIt2Servo
# from linkattacher_msgs.srv import AttachLink
# from linkattacher_msgs.srv import DetachLink
from ur_msgs.srv import SetIO

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import yaml
from std_srvs.srv import SetBool
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

# signal: to signal when to look for new transforms
signal = True 

# aruco_transforms: storing aruco Transforms
aruco_transforms = []

# if the flag is False then EEF goes back to default/resting position if no aruco box detected
flag = True

# task_queue: contains positions to be executed in FIFO manner
task_queue = []

# task_ptr: points to the current position under execution or that needs to be executed next
task_ptr = 0

# # srv: shows whether or not a request for payload has been received from the ebot
srv = True

# placed: this flag shows whether or not the box is placed on the ebot
placed = False

# box_aruco_frame: contains the payload/box frame_id picked up by the arm 
box_aruco_frame = ""


# ForceStatus
# fstat: stores force status of arm
fstat = 0

# netWrench: stores EEF force status
netWrench = 0.0

# task_done: check if task done or not for a box number (0 - not done && 1 - done)
task_done = [0]*15


# rBoxPose: to store box_name, left box pose and its quaternion 
lBoxPose = {
    "position": [],     # eg. "Lbox1", x, y, z
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094]
}

# rBoxPose: to store box_name, right box pose and its quaternion 
rBoxPose = {
    "position": [],     # eg. "Rbox1", x, y, z
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094]
}

# EEF_link: to store EEF pose, quat and euler angles
EEF_link = {
    "position": [0.0129, 0.5498, 0.1333],
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094],
    "euler_angles": [0, 0, 0]
}

# ur5 configs: to store different ur5 configuration (xyz and orientation)
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
        "position": ["rbTopPose", 0.1117, -0.4626, 0.4579],
        "quaternion": [0.71035, 0.70383, 0.00303, -0.00292]
    },

    "lbTopPose": {
        "position": ["lbTopPose", -0.1077, 0.4635, 0.4579],
        "quaternion": [0.71035, 0.70383, 0.00303, -0.00292]
    },

}

class Services(Node):
    def __init__(self):
        super().__init__("Tf_Finder")

        self.srv = self.create_service(SetBool, "/passing_service", self.handle_request)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def handle_request(self, request, response):
        '''
        handles payload requests from ebot and activates the arm 

        Input Arguments:
        ---
        `request` :  [ Bool ]
            request sent by ebot

        `response` :  [ bool ]
            response to ebot

        Returns:
        ---
        response
        '''

        global srv, placed

        # this signifies that a request is received and only then it will allow MoveItJointControl class's method servo motioin to control the arm
        srv = True

        # this shows that the box is not yet placed on the ebot
        placed = False
        self.get_logger().info('Incoming request\nreceive: %d' % (request.data))

        rate = self.create_rate(2, self.get_clock())

        while not placed:
            self.vel=Twist()
            self.vel.linear.x=0.0
            self.vel.angular.z=0.0
            self.cmd_vel_pub.publish(self.vel)
            rate.sleep()

        response.success = True
        response.message = box_aruco_frame
        print("response returned")
        return response


class TfFinder(Node):
    def __init__(self):
        super().__init__("Tf_Finder")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.EEF_target_frame = "wrist_3_link"   # EEF frame
        self.box_target_frame = "1226_base_"     # box aruco frame
        self.source_frame = "base_link"          # manipulator base link frame
        self.ebot_aruco_frame = "1226_base_6"    # ebot aruco frame
        self.offset = 0.08

        # task_done: check if task done or not for a box number (0 - not done && 1 - done)
        # self.task_done = [0]*15

        # pointers to list's indexes, to schedule tasks
        self.left_pose_ptr = -1
        self.right_pose_ptr = -1

        self.callback_group = ReentrantCallbackGroup()

        self.transform_checker = self.create_timer(0.01, self.check_transform)
        self.create_subscription(Int64, "/servo_node/ForceStatus", self.force_status_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Float64, "/net_wrench", self.EFF_nforce_callback, 10, callback_group=self.callback_group)


    def force_status_callback(self, msg:Int64):
        global fstat
        fstat = msg.data

    def EFF_nforce_callback(self, msg:Float64):
        global netWrench
        netWrench = msg.data

    # checking all necessary transforms
    def check_transform(self):
        '''
        Purpose:
        ---
        checks Transforms between base_link & EEF and update the EEF_link dict.

        checks Transforms between base_link & boxes, update lBoxPose/rBoxpose dict and schedule task to pick up the box. 

        Input Arguments:
        ---
        None

        Returns:
        ---
        NonerBoxPose

        Example call:
        ---
        the function is called every 0.01 sec by ros2 timer
        '''

        global aruco_transforms, signal, flag, task_done

        try:
            # feedback
            EEF_to_base = self.tf_buffer.lookup_transform(     # EEF w.r.t. base_link
                self.source_frame,
                self.EEF_target_frame,
                rclpy.time.Time()
            )

            # check for ebot aruco marker when received a request from ebot
            if srv:
                try:
                    ebot_aruco = self.tf_buffer.lookup_transform(     # ebot aruco w.r.t. base_link
                        self.source_frame,
                        self.ebot_aruco_frame,
                        rclpy.time.Time()
                    )

                    # update drop config
                    ur5_configs["drop_config"]["position"][0] = "drop_config"
                    ur5_configs["drop_config"]["position"][1] = ebot_aruco.transform.translation.x + self.offset
                    ur5_configs["drop_config"]["position"][2] = ebot_aruco.transform.translation.y
                    ur5_configs["drop_config"]["position"][3] = -0.1

                except TransformException as ex:
                    # self.get_logger().info(f'Could not transform {self.ebot_aruco_frame} to {self.source_frame}: {ex}')
                    pass

            # base_link to box transforms (aruco transforms) only if signal = True (i.e. when aruco_transform list is empty)
            if signal:
                # this function call updates the list aruco_transforms
                self.get_all_frames()

                if len(aruco_transforms) > 0: 
                    flag = False
                    signal = False

                    for tranform in aruco_transforms:
                        box_num = int(tranform.replace("1226_base_", ""))

                        # if not self.task_done[box_num]:
                        if not task_done[box_num]:
                            print(f"printing transform = {tranform}")
                            base_to_box = self.tf_buffer.lookup_transform(
                                                    "base_link",
                                                    tranform,
                                                    rclpy.time.Time())
                            
                            # if the Y coordinate of the box is greater than zero, this means that the box is on the left of the ur5 arm
                            if base_to_box.transform.translation.y > 0:
                                self.left_pose_ptr += 1
                                box_name = "Lbox" + tranform.strip("1226_base_")
                                # update the position key of lBoxPose
                                lst = [box_name, base_to_box.transform.translation.x, base_to_box.transform.translation.y, base_to_box.transform.translation.z]
                                lBoxPose["position"].append(lst)

                                self.schedule_tasks(box_pose=lBoxPose["position"][self.left_pose_ptr])

                            else:
                                self.right_pose_ptr += 1
                                box_name = "Rbox" + tranform.strip("1226_base_")
                                lst = [box_name, base_to_box.transform.translation.x, base_to_box.transform.translation.y, base_to_box.transform.translation.z]

                                rBoxPose["position"].append(lst)
                                self.schedule_tasks(box_pose=rBoxPose["position"][self.right_pose_ptr])

                            # this shows that task for box_num is done
                            # self.task_done[box_num] = 1
                            task_done[box_num] = 1

                # this means that no boxes are present
                elif not flag:
                    # goes back to the default position
                    self.schedule_tasks(end=True)
                    flag = True
            
            # updating EEF position and orientation for PID controller feedback 
            EEF_link["position"] = [EEF_to_base.transform.translation.x, EEF_to_base.transform.translation.y, EEF_to_base.transform.translation.z]
            EEF_link["quaternion"] = [EEF_to_base.transform.rotation.x, EEF_to_base.transform.rotation.y, EEF_to_base.transform.rotation.z, EEF_to_base.transform.rotation.w]
            EEF_link["euler_angles"] = tf_transformations.euler_from_quaternion(EEF_link["quaternion"])
                        
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.EEF_target_frame} to {self.source_frame}: {ex}')
            
    # scheduling task
    def schedule_tasks(self, box_pose=None, end=False):
        '''
        Purpose:
        ---
        adds various positions goals in task_queue.

        Input Arguments:
        ---
        `box_pose` :  [ list ]
            box location coordinates ([x, y, z])

        `end` :  [ bool ]
            if no more aruco detected, then end = True

        Returns:
        ---
        None

        Example call:
        ---
        self.schedule_tasks(box_pose=rBoxPose["position"])
        '''


        # for box_pos in box_poses:
        if box_pose != None:

            # if the Y coordinate of the box is greater than zero, this means that the box is on the left of the ur5 arm 
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
            task_queue.append(ur5_configs["start_config"]["position"])
        
        if end:
            # make ur5 go back to the default position and orientation
            task_queue.append(ur5_configs["default_config"]["position"])
            

    # get aruco frame names
    def get_all_frames(self):
        '''
        Purpose:
        ---
        provides all the box frame names (eg. obj_1) and append it to aruco_transforms (list).

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.get_all_frames()
        '''
        global task_done

        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        frames_dict = yaml.safe_load(frames_yaml)

        for frame in frames_dict:
            if frame.startswith("1226_base_"):
                if frame != "1226_base_6":
                    frame_id = int(frame.replace("1226_base_", ""))

                    # if not self.task_done[frame_id]:
                    if not task_done[frame_id]:
                        aruco_transforms.append(frame)


class MoveItJointControl(Node):
    def __init__(self):
        super().__init__("joint_controller")

        # max velocities
        self.max_lin_vel = 5.0
        self.max_ang_vel = 5.0

        # box placed
        self.box_placed = False

        # previous error (part of PID controller)
        self.prev_error = 0.0

        # execution flag to signal when to align the EEF in a vertical orientation
        self.execute = True    

        # box attached to EEF
        self.box_attached = ""

        # EEF force threshold
        self.force_threshold = 70.0
        self.on_air = 40.0
        self.is_box_attached = False

        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 Servo interface
        self.moveit2_servo = MoveIt2Servo(
            node=self,
            frame_id=ur5.base_link_name(),
            callback_group=self.callback_group,
            enable_at_init=True
        )

        # timer for servo motion
        # self.servoing = self.create_timer(0.001, self.servo_motion)
        self.servoing = self.create_timer(0.05, self.servo_motion)


        self.gripper_control = self.create_client(SetIO, '/io_and_status_controller/set_io')

        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF Tool service not available, waiting again...')

    def gripper_call(self, state):
        '''
        based on the state given as i/p the service is called to activate/deactivate
        pin 16 of TCP in UR5
        i/p: node, state of pin:Bool
        o/p or return: response from service call
        '''
        req         = SetIO.Request()
        req.fun     = 1
        req.pin     = 16
        req.state   = float(state)
        
        future = self.gripper_control.call_async(req)
        future.add_done_callback(self.gripper_callback)
        print(f"request for gripper state = {state} is sent!!")

    def gripper_callback(self, future):
        try:
            response = future.result()
            print(f"response from /io_and_status_controller/set_io: {response}")
        except Exception as e:
            print(f"error: {e}")

    # PID control
    def PID_controller(self, error, Kp, Kd = 0, Ki = 0):
        '''
        Purpose:
        ---
        calculate the velocity required by the EEF to reach the goal efficiently. 

        Input Arguments:
        ---
        `error` :  [ float ]
            difference between goal location and current location 

        `Kp` :  [ float ]
            weight for proportional part

        `Kd` :  [ float ]
            weight for derivative part

        `Ki` :  [ float ]
            weight for integral part

        Returns:
        ---
        `vel` :  [ float ]
            velocity of the EEF

        Example call:
        ---
        self.PID_controller(error=error_x, Kp=4.3)
        '''

        vel = (Kp * error) + (-Kd * (self.prev_error-error)) + (Ki * 1)
        vel = max(min(vel, self.max_lin_vel), -self.max_lin_vel) 
        self.prev_error = error
        return round(vel, 4)

    # checking if goal reached (with tolerance)
    def goal_reached(self, error, tolerance = 0.01):
        '''
        Purpose:
        ---
        To check if goal is reached.

        Input Arguments:
        ---
        `error` :  [ float ]
            difference between goal location and current location 

        `tolerance` :  [ float ]
            tolerance

        Returns:
        ---
        bool

        Example call:
        ---
        self.goal_reached(error_x, tolerance=0.009)
        '''

        if abs(error) <= tolerance:
            return True
        
        return False

    # ur5 controller
    def servo_motion(self):
        '''
        Purpose:
        ---
        timer callback to control the velocity of the EEF.

        Input Arguments:
        ---
        None

        Returns:
        ---
        None
        '''

        global task_ptr, signal, srv, placed, box_aruco_frame, netWrench, task_done

        if len(task_queue):
            if self.execute:
                # PID control for EEF orientation
                error_ang_x = ur5_configs["start_config"]["euler_angles"][0] - EEF_link["euler_angles"][0]
                ang_vel_Y = self.PID_controller(error=error_ang_x, Kp=10.0)
                
                self.moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, ang_vel_Y, 0.0))

                if self.goal_reached(error_ang_x, tolerance=0.1):
                    self.execute = False
                
            else:
                # only if srv is true, which is when there's a request on the service /passing_service, execute the rest of the logic
                if srv:                
                    # PID control for EEF position
                    error_x = task_queue[task_ptr][1] - EEF_link["position"][0]
                    error_y = task_queue[task_ptr][2] - EEF_link["position"][1]
                    error_z = task_queue[task_ptr][3] - EEF_link["position"][2]

                    # self.is_box_attached = False
                    
                    # checking if the goal is reached
                    # check which one to use "or" or "and" in if condition
                    if ((self.goal_reached(error_x, tolerance=0.02) and self.goal_reached(error_y, tolerance=0.02) and self.goal_reached(error_z, tolerance=0.02)) or (netWrench >= self.force_threshold)):
                        self.box_attached = ""
                        if (task_queue[task_ptr][0][0] == "L") or (task_queue[task_ptr][0][0] == "R"):
                            self.box_attached = task_queue[task_ptr][0][1:]
                            self.gripper_call(1.0)
                            self.is_box_attached = True
                            print("self.is_box_attached = True")

                        elif task_queue[task_ptr][0] == "drop_config":
                            self.gripper_call(0.0)

                            # updating this to True, returns the response to the client with a message that the box is placed on the ebot
                            placed = True

                            box_aruco_frame = self.box_attached

                            if len(aruco_transforms) == 0:
                                # if aruco transforms is empty, then updating signal to true means to look up for any new aruco (box) transforms
                                signal = True
                            else:
                                # removing transforms when the task is done
                                aruco_transforms.pop(0)
                            
                            self.box_placed = True

                        # ****************************************************************************************
                        elif ((task_queue[task_ptr][0] == "rbTopPose") or (task_queue[task_ptr][0] == "lbTopPose") and self.is_box_attached):
                            if netWrench <= self.on_air:
                                self.no_box = True
                                task_done[int(self.box_attached[-1])] = 0
                                task_ptr += 2

                                self.is_box_attached = False
                                aruco_transforms.pop(0)

                        elif self.box_placed and task_queue[task_ptr][0] == "start_config":
                            # once the box is dropped, wait until the client again requests on the /passing_service
                            srv = False
                            self.box_placed = False

                        if task_ptr < len(task_queue)-1: task_ptr += 1
                        if task_ptr >= len(task_queue): task_ptr = len(task_queue)-1

                    ln_vel_X = self.PID_controller(error=error_x, Kp=10.0)
                    ln_vel_Y = self.PID_controller(error=error_y, Kp=10.0)
                    ln_vel_Z = self.PID_controller(error=error_z, Kp=15.0)
                        
                    self.moveit2_servo(linear=(ln_vel_X, ln_vel_Y, ln_vel_Z), angular=(0.0, 0.0, 0.0)) 


def main(args=None):
    '''
    Purpose:
    ---
    Initialize ROS communications for a given context.

    Add Nodes to the MultiThreadedExecutor whose callbacks should be managed parallely.

    Shuts down the previously initialized context.
    
    Input Arguments:
    ---
    None
    
    Returns:
    ---
    None
    '''

    rclpy.init(args=args)

    TfFinderNode = TfFinder()
    JointControlNode = MoveItJointControl()
    services = Services()

    executor = MultiThreadedExecutor()
    executor.add_node(TfFinderNode)
    executor.add_node(JointControlNode)
    executor.add_node(services)
    executor.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()