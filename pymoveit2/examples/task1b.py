#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
*****************************************************************************************
*
*        	===============================================
*           		    Logistic coBot (LB) Theme (eYRC 2024-25)
*        	===============================================
*
*  This script implements Task 1B of the Logistic coBot (LB) Theme (eYRC 2024-25).
*
*****************************************************************************************
'''

# Team ID:          [ LB#1226 ]
# Author List:		[ Athrva Kulkani]
# Filename:		    task1b.py
# Functions:		[ calculate_rectangle_area, detect_aruco, aruco_tf, main ]
# Nodes:		    Publishing Topics  - [ /tf ]
#                   Subscribing Topics - [ /camera/color/image_raw, /camera/aligned_depth_to_color/image_raw ]

################### IMPORT MODULES #######################
import math
import rclpy
import cv2
import numpy as np
import tf2_ros
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import tf_transformations
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import CompressedImage 
from sensor_msgs.msg import Image


##################### FUNCTION DEFINITIONS #######################
def rotation_matrix_to_euler_angles(R):
    # Check if pitch is close to ±90 degrees
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw = math.atan2(R[1,0], R[0,0])
    else:
        roll = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw = 0

    return np.array([roll, pitch, yaw])

def calculate_rectangle_area(coordinates):
    '''
    Calculate area of detected ArUco marker.

    Args:
        coordinates (list): Coordinates of detected ArUco (4 set of (x,y) coordinates)

    Returns:
        area (float): Area of detected ArUco
        width (float): Width of detected ArUco
    '''
    top_left, top_right, bottom_right, bottom_left = coordinates

    # Calculate width and height
    width = np.linalg.norm(top_right - top_left)
    height = np.linalg.norm(bottom_left - top_left)

    # Calculate area
    area = width * height

    return area, width

def detect_aruco(image, cam_mat, dist_mat):
    '''
    Detect ArUco markers and return their details.

    Args:
        image (Image): Input image frame from camera
        cam_mat (ndarray): Camera matrix for pose estimation
        dist_mat (ndarray): Distortion coefficients for pose estimation

    Returns:
        center_aruco_list (list): Center points of detected ArUco markers
        distance_from_rgb_list (list): Distance values of detected ArUco markers (use depth image instead)
        angle_aruco_list (list): Angles of detected ArUco markers
        width_aruco_list (list): Widths of detected ArUco markers
        ids (list): List of detected ArUco marker IDs
        markerCorners (list): Corners of detected ArUco markers
    '''
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(gray_image)

    center_aruco_list = []
    distance_from_rgb_list = []  # No longer used for RGB; now depth will be used
    angle_aruco_list = []
    angle_aruco_list_1 = []
    angle_aruco_list_2 = []
    width_aruco_list = []

    if markerIds is not None:
        for i in range(len(markerIds)):
            area, width = calculate_rectangle_area(markerCorners[i][0])
            if area < 1500:
                continue

            center_x = int(np.mean(markerCorners[i][0][:, 0]))
            center_y = int(np.mean(markerCorners[i][0][:, 1]))
            center_aruco_list.append((center_x, center_y))

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners[i], 0.15, cam_mat, dist_mat)
            
            angle_aruco = (0.788 * rvec[0][0][0]) - ((rvec[0][0][0] ** 2) / 3160)
            angle_aruco_1 = (0.788 * rvec[0][0][1]) - ((rvec[0][0][1] ** 2) / 3160)
            angle_aruco_2 = (0.788 * rvec[0][0][2]) - ((rvec[0][0][2] ** 2) / 3160)
           
            
            # print(f"raw quaternion values {rvec}")
            angle_aruco_list.append(angle_aruco)
            angle_aruco_list_1.append(angle_aruco_1)
            angle_aruco_list_2.append(angle_aruco_2)

            # No longer using this for distance (depth will be calculated later)
            distance_from_rgb_list.append(tvec[0][0][2])

            width_aruco_list.append(width)
            # cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list,angle_aruco_list_1,angle_aruco_list_2, width_aruco_list, markerIds, markerCorners

##################### CLASS DEFINITION #######################

def quaternion_multiply(q0, q1):
    """
    Multiplies two quaternions.

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion
class aruco_tf(Node):
    '''
    Class to detect ArUco markers and publish TFs for pose estimation.
    '''

    def __init__(self):
        '''
        Initialization of class aruco_tf
        '''
        super().__init__('aruco_tf_publisher')  # Registering node
        self.get_logger().info("Node Started")


        self.color_cam_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)
        self.publisher_ = self.create_publisher(CompressedImage, '/camera/image/imageLB1226', 10)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.2, self.process_image)

        self.cv_image = None
        self.depth_image = None
  
    def depthimagecb(self, data):
        '''
        Callback function for aligned depth camera topic.
        '''
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.get_logger().info(f'got the depth image: {e}')

        except CvBridgeError as e:
            self.get_logger().info(f'Error converting depth image')

    def colorimagecb(self, data):
        '''
        Callback function for colour camera raw topic.
        '''
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.get_logger().error(f'got the colored image')

        except CvBridgeError as e:
            self.get_logger().error(f'Error converting color image: {e}')

    def process_image(self):
        '''
        Timer function used to detect ArUco markers and publish TF.
        '''
        if self.cv_image is None :
            self.get_logger().info("colored not  image recevied")
            return
        if self.depth_image is None :
            self.get_logger().info("depth not image recevied")
            return

        cam_mat = np.array([[931.1829833984375, 0, 640],
                            [0, 931.1829833984375, 360],
                            [0, 0, 1]])
        dist_mat = np.zeros((1, 5))  # Modify based on camera calibration

        self.get_logger().info("moving to calculations part")
        center_aruco_list, distance_from_rgb_list, angle_aruco_list,angle_aruco_list_1,angle_aruco_list_2, width_aruco_list, ids, markerCorners = detect_aruco(self.cv_image, cam_mat, dist_mat)

        if ids is not None and len(center_aruco_list) == len(ids):  # Add length check
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners[i], 0.15, cam_mat, dist_mat)
                cv2.drawFrameAxes(self.cv_image, cam_mat, dist_mat, rvec, tvec, 0.9)
                
                # angle_aruco = (0.788*rvec[0][0][2]) - ((rvec[0][0][2]**2)/3160)
                rotationmatrix, _ = cv2.Rodrigues(rvec[0][0])
                angle_aruco = rotation_matrix_to_euler_angles(rotationmatrix)
                angle_aruco = angle_aruco[2]
                
                print(f"angle aruco {angle_aruco}")

                # print(angle_aruco_list)
                if i < len(angle_aruco_list):
                    corrected_angle = angle_aruco_list[i]
                    corrected_angle_1 = angle_aruco_list_1[i]
                    corrected_angle_2 = angle_aruco_list_2[i]
                else:
                    # Handle the case where `i` is out of range, e.g., log a message or set a default value
                    corrected_angle = 0  # or some default value
                    corrected_angle_1 = 0  # or some default value
                    corrected_angle_2 = 0  # or some default value
                    print(f"Warning: Index {i} out of range for angle_aruco_list")
                
                # print(corrected_angle)
               
                # print(f"current id is : {ids[i]}")
            # Convert the corrected yaw angle to quaternions
                # quaternion = R.from_euler('xyz', [roll, pitch, corrected_angle+yaw_offset]).as_quat()
                # quaternion_1 = R.from_euler('zyx', [roll, pitch, corrected_angle_1+yaw_offset]).as_quat()
                # quaternion_2 = R.from_euler('xyz', [roll, pitch, corrected_angle_2+yaw_offset]).as_quat()
                # print(f"quaternion 1:{quaternion} and quaternion 2 {quaternion_1} and quaternion 3 : {quaternion_2}")
                cX, cY = center_aruco_list[i]

                # Fetch the depth from the depth image at the ArUco marker's center
                depth_value = self.depth_image[cY, cX]  # Assuming depth image has   the same dimensions as RGB

                if depth_value == 0:  # Handle missing or invalid depth
                    self.get_logger().warn(f'Invalid depth at ArUco center: {cX}, {cY}')
                    continue

                distance_from_depth = depth_value /1000.0# Convert depth to meters
                # print(distance_from_depth)
                # Rectify coordinates
                y = distance_from_depth * (1280 - cX - 640) / 931.1829833984375
                z = distance_from_depth * (720 - cY - 360) / 931.1829833984375
                x = distance_from_depth 

                cv2.circle(self.cv_image, (cX, cY), 5, (0, 255, 0), -1)
                # z_rotation = R.from_euler('zxy', [np.pi/2,0,0]).as_quat()  # 90 degrees = π/2 radians
                # y_rotation = R.from_euler('z', np.pi/2).as_quat()  # 90 degrees = π/2 radians
                # combined_rotation = R.from_quat(z_rotation) * R.from_quat(quaternion_2) 
                # combined_quaterion =combined_rotation.as_quat()
                # print(f"values after combined rotation {combined_quaterion}")
                # quaternion_2 = tf_transformations.quaternion_from_euler(roll, pitch, corrected_angle_2)
                # z_rotation= tf_transformations.quaternion_from_euler(0.0,-0.0, -np.pi/2)
                # x_rotation= tf_transformations.quaternion_from_euler(0,np.pi/2.2, 0.)
                # # fix_rotation= tf_transformations.quaternion_from_euler(0,np.pi*-0.1, 0)
                # quaternion_2 =quaternion_multiply(quaternion_2,z_rotation)
                # quaternion_2 =quaternion_multiply(quaternion_2,x_rotation)
                # # quaternion_2 =quaternion_multiply(quaternion_2,fix_rotation)
                # print(f"values after combined rotation with function {quaternion_2}")

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_link'
                t.child_frame_id = f'cam_{ids[i][0]}'
                # print(f"{t.child_frame_id }= f'obj_{ids[i][0]}'")
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = z
                # print(f"x,y,z { t.transform.translation.x},{ t.transform.translation.y},{ t.transform.translation.z}")
                # rmat, _ = cv2.Rodrigues(rvec[0][0])
                # r = R.from_matrix(rmat)

                # # Convert the rotation matrix to quaternion
                # quaternion = r.as_quat()

                            # Combination 1_1
   # Original roll, pitch, yaw values
                roll = np.pi/2
                pitch = -angle_aruco
                yaw = np.pi/2

                # Initialize quaternion array
                quaternion = [0.0, 0.0, 0.0, 0.0]

                # Calculate quaternion components directly
                try:
                
                    quaternion=tf_transformations.quaternion_from_euler(roll, pitch, yaw)
                    print(f"quaternions {quaternion}")
                except Exception as e:
                    self.get_logger().error(f"Error calculating quaternion: {str(e)}")
                    continue

                try:
                    # Convert quaternion to scipy rotation
                    quaternion = R.from_quat(quaternion)
                    offset = R.from_euler('y', np.pi/4-0.028, degrees=False)
                    quaternion_2 = offset * quaternion
                    quaternion_2 = quaternion_2.as_quat()
                    print(f"adjusted quaternions {quaternion_2}")

                except Exception as e:
                    self.get_logger().error(f"Error applying rotation correction: {str(e)}")
                    continue

                # Apply quaternion to transform
                t.transform.rotation.x = float(quaternion_2[0])
                t.transform.rotation.y = float(quaternion_2[1]) 
                t.transform.rotation.z = float(quaternion_2[2])
                t.transform.rotation.w = float(quaternion_2[3])

                self.br.sendTransform(t)
    
                try:
                    trans = self.tf_buffer.lookup_transform('base_link', f'cam_{ids[i][0]}', rclpy.time.Time())
                    obj_t = TransformStamped()
                    obj_t.header.stamp = self.get_clock().now().to_msg()
                    obj_t.header.frame_id = 'base_link'
                    obj_t.child_frame_id = f'1226_base_{ids[i][0]}'
                    obj_t.transform.translation = trans.transform.translation
                    # quaternion_2 = tf_transformations.quaternion_from_euler(0,0, -angle_aruco)
                    # # print(f"quaternions for {quaternion_2}")
                    
                    


                    # # Convert roll, pitch, yaw to quaternion
                    # quaternion_initial = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
                    # quaternion=[0,0,0,0]
                    # # quaternion[0]=quaternion_initial[1]
                    # # quaternion[1]=quaternion_initial[2]
                    # # quaternion[2]=quaternion_initial[3]
                    # # quaternion[3]=quaternion_initial[0]

                    

                    # # # Use quaternion multiplication to combine the initial quaternion and the correction
                    # print(f"quaternions {quaternion}")
                    
                   
                    obj_t.transform.rotation = trans.transform.rotation
                    self.br.sendTransform(obj_t)
                except Exception as e:
                    self.get_logger().warn(f"Transform lookup failed: {str(e)}")

            compressed_image = self.bridge.cv2_to_compressed_imgmsg(self.cv_image, dst_format='jpeg')
            self.publisher_.publish(compressed_image)
            # cv2.imshow('Detected ArUco markers with Axes', self.cv_image)
            cv2.waitKey(1)

##################### MAIN FUNCTION #######################


def main(args=None):
    rclpy.init(args=args)
    node = aruco_tf()   
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    
    
    