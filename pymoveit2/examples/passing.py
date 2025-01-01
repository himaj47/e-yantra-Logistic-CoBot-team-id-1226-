'''
# Team ID:          1226
# Theme:            Logistic coBot
# Author List:      Himaj Joshi
# Filename:         passing.py
# Functions:        __init__, handle_request, check_transform, schedule_tasks, get_all_frames, mag_on_callback, mag_off_callback, rm_box_callback, magnet_on, magnet_off, PID_controller, goal_reached, servo_motion, main
# Global variables: signal, aruco_transforms, flag, task_queue, task_ptr, srv, placed, lBoxPose, rBoxPose, EEF_link, ur5_configs
'''

#! /usr/bin/env python3
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped
import rclpy.time

from pymoveit2.robots import ur5
import tf_transformations

from pymoveit2 import MoveIt2Servo
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import yaml
from std_srvs.srv import SetBool
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

# service handler flags
srv = False
placed = False

# aruco frame
aruco_frame = ""


# rBoxPose: to store box_name, left box pose and its quaternion 
lBoxPose = {
    "box_name": "box2",
    "position": ["lBoxPose", 0.0, 0.0, 0.0],
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094]
}

# rBoxPose: to store box_name, right box pose and its quaternion 
rBoxPose = {
    "box_name": "box3",
    "position": ["rBoxPose", 0.0, 0.0, 0.0],
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094]
}

# # eBotPose: to store ebot pose and orientation in quaternion 
# eBotPose = {
#     "position": ["rBoxPose", 0.0, 0.0, 0.0],
#     "quaternion": [0.9992, 0.0382, -0.0035, 0.0094]
# }

# EEF_link: to store EEF pose, quat and euler angles
EEF_link = {
    "position": [0.0129, 0.5498, 0.1333],
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094],
    "euler_angles": [0, 0, 0]
}

# ur5 configs: to store different ur5 configuration 
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
        global srv
        global placed

        # this signifies that a request is received
        srv = True

        # this shows whether the box is placed on the ebot or not
        placed = False
        self.get_logger().info('Incoming request\nreceive: %d' % (request.data))

        rate = self.create_rate(2, self.get_clock())

        while not placed:
            self.get_logger().info("Waiting for the box to be placed ...")
            self.vel=Twist()
            self.vel.linear.x=0.0
            self.vel.angular.z=0.0
            self.cmd_vel_pub.publish(self.vel)
            rate.sleep()

        response.success = True
        response.message = aruco_frame
        return response


class TfFinder(Node):
    def __init__(self):
        super().__init__("Tf_Finder")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.EEF_target_frame = "wrist_3_link"   # EEF frame
        self.box_target_frame = "obj_"           # box frame
        self.source_frame = "base_link"
        self.ebot_aruco_frame = "obj_12"
        self.offset = 0.08

        # task_done: check if task done or not for a box number (0 - not done && 1 - done)
        self.task_done = [0]*15

        self.callback_group = ReentrantCallbackGroup()

        self.transform_checker = self.create_timer(0.01, self.check_transform)
        

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
        None

        Example call:
        ---
        the function is called every 0.01 sec by ros2 timer
        '''

        global aruco_transforms
        global signal
        global flag

        try:
            # feedback
            EEF_to_base = self.tf_buffer.lookup_transform(     # EEF w.r.t. base_link
                self.source_frame,
                self.EEF_target_frame,
                rclpy.time.Time()
            )

            # check for ebot aruco marker
            if srv:
                try:
                    ebot_aruco = self.tf_buffer.lookup_transform(     # EEF w.r.t. base_link
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
                    self.get_logger().info(f'Could not transform {self.ebot_aruco_frame} to {self.source_frame}: {ex}')
                

            # base_link to box transforms (aruco transforms)
            if signal:
                self.get_all_frames()

                if len(aruco_transforms) > 0:
                    flag = False
                    signal = False

                    for tranform in aruco_transforms:
                        box_num = int(tranform.strip("obj_"))

                        if not self.task_done[box_num]:
                            base_to_box = self.tf_buffer.lookup_transform(
                                                    "base_link",
                                                    tranform,
                                                    rclpy.time.Time())
                            
                            # if the Y coordinate of the box is greater than zero, this means that the box is on the left of the ur5 arm
                            if base_to_box.transform.translation.y > 0:
                                # update the position key of lBoxPose
                                lBoxPose["position"][0] = "lBoxPose" 
                                lBoxPose["position"][1] = base_to_box.transform.translation.x
                                lBoxPose["position"][2] = base_to_box.transform.translation.y
                                lBoxPose["position"][3] = base_to_box.transform.translation.z
                                lBoxPose["box_name"] = "box" + tranform.strip("obj_")
                                self.schedule_tasks(box_pose=lBoxPose["position"])
                            else:
                                # update the position key of rBoxPose
                                rBoxPose["position"][0] = "rBoxPose"
                                rBoxPose["position"][1] = base_to_box.transform.translation.x 
                                rBoxPose["position"][2] = base_to_box.transform.translation.y
                                rBoxPose["position"][3] = base_to_box.transform.translation.z
                                rBoxPose["box_name"] = "box" + tranform.strip("obj_")
                                self.schedule_tasks(box_pose=rBoxPose["position"])

                            self.task_done[box_num] = 1

                elif not flag:
                    self.schedule_tasks(end=True)
                    flag = True
            
            # self.get_logger().info(str(EEF_to_base.transform.translation))
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
            task_queue.append(ur5_configs["start_config"]["position"])

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
        
        if end:
            # make ur5 go back to the default position and orientation
            task_queue.append(ur5_configs["start_config"]["position"])
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

        frames_yaml = self.tf_buffer.all_frames_as_yaml()
        frames_dict = yaml.safe_load(frames_yaml)

        for frame in frames_dict:
            if frame.startswith("obj_"):
                if frame != "obj_12":
                    frame_id = int(frame.strip("obj_"))

                    if not self.task_done[frame_id]:
                        aruco_transforms.append(frame)


class MoveItJointControl(Node):
    def __init__(self):
        super().__init__("joint_controller")

        # max velocities
        self.max_lin_vel = 5.0
        self.max_ang_vel = 5.0

        # previous error (part of PID controller)
        self.prev_error = 0.0

        # execution flag to signal when to align the EEF in a vertical orientation
        self.execute = True    

        # box attached to EEF
        self.box_attached = ""

        self.callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 Servo interface
        self.moveit2_servo = MoveIt2Servo(
            node=self,
            frame_id=ur5.base_link_name(),
            callback_group=self.callback_group,
            enable_at_init=True
        )

        # timer for servo motion
        self.servoing = self.create_timer(0.001, self.servo_motion)

        # clients 
        self.gripper_control_attach = self.create_client(AttachLink, '/GripperMagnetON')
        self.gripper_control_detach = self.create_client(DetachLink, '/GripperMagnetOFF')

        while not self.gripper_control_attach.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF /GripperMagnetON service not available, waiting again...')

        while not self.gripper_control_detach.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service /GripperMagnetOFF not available, waiting again...')

    # callbacks
    def mag_on_callback(self, future):
        try:
            response = future.result()
            # print(f"response from /GripperON: {response}")
        except Exception as e:
            print("error: {e}")

    def mag_off_callback(self, future):
        try:
            response = future.result()
            # print(f"response from /GripperOFF: {response}")
        except Exception as e:
            print("error: {e}")

    def rm_box_callback(self, future):
        try:
            response = future.result()
            # print(f"response from /SERVOLINK: {response}")
        except Exception as e:
            print("error: {e}")

    # clients
    def magnet_on(self, box_name: str | None):
        req = AttachLink.Request()
        req.model1_name =  box_name     
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        future = self.gripper_control_attach.call_async(req)
        future.add_done_callback(self.mag_on_callback)
        # print(f"request for magnet on for box {box_name} is sent!!")

    def magnet_off(self, box_name: str | None):
        req = DetachLink.Request()
        req.model1_name =  box_name     
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        future = self.gripper_control_detach.call_async(req)
        future.add_done_callback(self.mag_off_callback)
        # print(f"request for magnet off for box {box_name} is sent!!")

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
    def goal_reached(self, error, tolerance = 0.005):
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

        global task_ptr
        global signal
        global srv
        global placed
        global aruco_frame

        if len(task_queue):
            if self.execute:
                # PID control for EEF orientation
                error_ang_x = ur5_configs["start_config"]["euler_angles"][0] - EEF_link["euler_angles"][0]
                ang_vel_Y = self.PID_controller(error=error_ang_x, Kp=4.3)
                
                self.moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, ang_vel_Y, 0.0))

                if self.goal_reached(error_ang_x, tolerance=0.008):
                    self.execute = False
                
            else:
                # only if srv is true, which is when there's a request on the service /passing_service, execute the rest of the logic
                if srv:
                    # PID control for EEF position
                    error_x = task_queue[task_ptr][1] - EEF_link["position"][0]
                    error_y = task_queue[task_ptr][2] - EEF_link["position"][1]
                    error_z = task_queue[task_ptr][3] - EEF_link["position"][2]
                    
                    # checking if the goal is reached
                    if (self.goal_reached(error_x, tolerance=0.009) and self.goal_reached(error_y, tolerance=0.009) and self.goal_reached(error_z, tolerance=0.009)):
                        if task_queue[task_ptr][0] == "lBoxPose":
                            self.box_attached = lBoxPose["box_name"]
                            self.magnet_on(lBoxPose["box_name"])

                        if task_queue[task_ptr][0] == "rBoxPose":
                            self.box_attached = rBoxPose["box_name"]
                            self.magnet_on(rBoxPose["box_name"])

                        if task_queue[task_ptr][0] == "drop_config":
                            print("box attached: " + str(self.box_attached))
                            self.magnet_off(box_name=self.box_attached)
                            
                            # once the box is dropped, wait until the client again requests on the /passing_service
                            srv = False

                            # updating this to True, returns the response to the client with a message that the box is placed on the ebot
                            placed = True

                            aruco_frame = self.box_attached

                            if len(aruco_transforms) == 0:
                                # if aruco transforms is empty, then updating signal to true means to look up for amy new aruco transforms
                                signal = True
                            else:
                                aruco_transforms.pop(0)

                        if task_ptr < len(task_queue)-1: task_ptr += 1

                    else:
                        ln_vel_X = self.PID_controller(error=error_x, Kp=4.3)
                        ln_vel_Y = self.PID_controller(error=error_y, Kp=4.3)
                        ln_vel_Z = self.PID_controller(error=error_z, Kp=4.3)

                        # print("ln_vel_X: " + str(ln_vel_X), "  ln_vel_Y: " + str(ln_vel_Y), "  ln_vel_Z: " + str(ln_vel_Z))
                        
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