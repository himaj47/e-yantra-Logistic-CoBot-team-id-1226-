#! /usr/bin/env python3
import math, time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TwistStamped
import rclpy.time
from rclpy.clock import Clock

from pymoveit2.robots import ur5
import tf_transformations

from pymoveit2 import MoveIt2Servo
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from servo_msgs.srv import ServoLink

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage

import yaml
# from request_payload.srv import Payload
from std_srvs.srv import SetBool

# signal flag
signal = True 

# storing aruco Transforms
aruco_transforms = []

# if the flag is False then EEF goes back to default/resting position if no aruco box detected
flag = True

# order of tasks
task_queue = []
task_ptr = 0

# service handler flag
srv = False
placed = False


# left box pose
lBoxPose = {
    "box_name": "box2",
    "position": ["lBoxPose", 0.0, 0.0, 0.0],
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094]
}

# right box pose 
rBoxPose = {
    "box_name": "box3",
    "position": ["rBoxPose", 0.0, 0.0, 0.0],
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094]
}

# EEF pose & quat
EEF_link = {
    "position": [0.0129, 0.5498, 0.1333],
    "quaternion": [0.9992, 0.0382, -0.0035, 0.0094],
    "euler_angles": [0, 0, 0]
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

    def handle_request(self, request, response):
        global srv
        global placed

        srv = True
        placed = False
        self.get_logger().info('Incoming request\nreceive: %d' % (request.data))

        rate = self.create_rate(2, self.get_clock())

        while not placed:
            self.get_logger().info("Waiting for the box to be placed ...")
            rate.sleep()

        response.success = True
        return response


class TfFinder(Node):
    def __init__(self):
        super().__init__("Tf_Finder")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.EEF_target_frame = "wrist_3_link"   # EEF frame
        self.box_target_frame = "obj_"           # box frame
        self.source_frame = "base_link"

        # check if task done or not for a box number (0 - not done && 1 - done)
        self.task_done = [0]*15

        self.callback_group = ReentrantCallbackGroup()

        self.transform_checker = self.create_timer(0.01, self.check_transform)
        

    # checking all necessary transforms
    def check_transform(self):
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
                        
                            if base_to_box.transform.translation.y > 0:
                                lBoxPose["position"][0] = "lBoxPose" 
                                lBoxPose["position"][1] = base_to_box.transform.translation.x
                                lBoxPose["position"][2] = base_to_box.transform.translation.y
                                lBoxPose["position"][3] = base_to_box.transform.translation.z
                                lBoxPose["box_name"] = "box" + tranform.strip("obj_")
                                self.schedule_tasks(box_pose=lBoxPose["position"])
                            else:
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
            
            # node.get_logger().info(str(EEF_to_base.transform.translation))
            EEF_link["position"] = [EEF_to_base.transform.translation.x, EEF_to_base.transform.translation.y, EEF_to_base.transform.translation.z]
            EEF_link["quaternion"] = [EEF_to_base.transform.rotation.x, EEF_to_base.transform.rotation.y, EEF_to_base.transform.rotation.z, EEF_to_base.transform.rotation.w]
            EEF_link["euler_angles"] = tf_transformations.euler_from_quaternion(EEF_link["quaternion"])
            EEF_to_base.transform

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.EEF_target_frame} to {self.source_frame}: {ex}')
            
    # scheduling task
    def schedule_tasks(self, box_pose=None, end=False):
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

            task_queue.append("placed")
        
        if end:
            task_queue.append(ur5_configs["start_config"]["position"])
            task_queue.append(ur5_configs["default_config"]["position"])
            

    # get aruco frame names
    def get_all_frames(self):
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
        self.prev_error = 0.0

        # execution flag
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
        # self.servo_control = self.create_client(ServoLink, '/SERVOLINK')

        while not self.gripper_control_attach.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF /GripperMagnetON service not available, waiting again...')

        while not self.gripper_control_detach.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service /GripperMagnetOFF not available, waiting again...')

        # while not self.servo_control.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Servo service not available, waiting again...')

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

    # def remove_box(self, box_name: str | None):
    #     req = ServoLink.Request()
    #     req.box_name =  box_name    
    #     req.box_link  = 'link'       
    #     future = self.servo_control.call_async(req)
    #     future.add_done_callback(self.rm_box_callback)

    # PID control
    def PID_controller(self, error, Kp, Kd = 0, Ki = 0):
        vel = (Kp * error) + (-Kd * (self.prev_error-error)) + (Ki * 1)
        vel = max(min(vel, self.max_lin_vel), -self.max_lin_vel)  # change this for angular too
        self.prev_error = error
        return round(vel, 4)

    # checking if goal reached (with tolerance)
    def goal_reached(self, error, tolerance = 0.005):
        if abs(error) <= tolerance:
            return True
        
        return False

    # ur5 controller
    def servo_motion(self):
        global task_ptr
        global signal
        global srv
        global placed

        if len(task_queue):
            if self.execute:
                # PID control for EEF orientation
                error_ang_x = ur5_configs["start_config"]["euler_angles"][0] - EEF_link["euler_angles"][0]
                ang_vel_Y = self.PID_controller(error=error_ang_x, Kp=4.3)
                
                self.moveit2_servo(linear=(0.0, 0.0, 0.0), angular=(0.0, ang_vel_Y, 0.0))

                if self.goal_reached(error_ang_x, tolerance=0.008):
                    self.execute = False
                
            else:
                if srv:
                    # PID control for EEF position
                    error_x = task_queue[task_ptr][1] - EEF_link["position"][0]
                    error_y = task_queue[task_ptr][2] - EEF_link["position"][1]
                    error_z = task_queue[task_ptr][3] - EEF_link["position"][2]

                    # print("error_x: " + str(error_x), "  error_y: " + str(error_y), "  error_z: " + str(error_z))

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
                            # self.remove_box(self.box_attached)

                            srv = False
                            placed = True

                            if len(aruco_transforms) == 0:
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