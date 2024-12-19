import numpy as np
from geometry_msgs.msg import Pose, Quaternion, Point
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Slerp, Rotation as R
import time

def linear_interpolation(start, end, steps):
    """Linear interpolation between start and end point"""
    return [start + (end - start) * t for t in np.linspace(0, 1, steps)]

def interpolate_quaternions(q_start, q_end, steps=10):
    start_rot = R.from_quat([q_start.x, q_start.y, q_start.z, q_start.w])
    end_rot = R.from_quat([q_end.x, q_end.y, q_end.z, q_end.w])

    t_values = np.linspace(0, 1, steps)
    slerp = Slerp([0, 1], R.from_quat([start_rot.as_quat(), end_rot.as_quat()]))
    interpolated_rots = slerp(t_values)

    return interpolated_rots.as_quat()

def interpolate_pose(start_pose, end_pose, steps=10):
    pos_x = linear_interpolation(start_pose.position.x, end_pose.position.x, steps)
    pos_y = linear_interpolation(start_pose.position.y, end_pose.position.y, steps)
    pos_z = linear_interpolation(start_pose.position.z, end_pose.position.z, steps)

    quaternions = interpolate_quaternions(start_pose.orientation, end_pose.orientation, steps)
    quat_x = quaternions[:, 0]
    quat_y = quaternions[:, 1]
    quat_z = quaternions[:, 2]
    quat_w = quaternions[:, 3]

    interpolated_poses = []
    for i in range(steps):
        pose = Pose()
        pose.position.x = pos_x[i]
        pose.position.y = pos_y[i]
        pose.position.z = pos_z[i]
        pose.orientation.x = quat_x[i]
        pose.orientation.y = quat_y[i]
        pose.orientation.z = quat_z[i]
        pose.orientation.w = quat_w[i]
        interpolated_poses.append(pose)

    return interpolated_poses

def main():
    rclpy.init()

    # Create node
    node = Node("interpolated_trajectory")

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
    )
    
      # Increase planning time and allow replanning
    # moveit2.set_planning_time(10.0)
    # moveit2.allow_replanning(True)
    # Define points
    start = {"position": [0.16, 0.10, 0.46], "orientation": [0.49, 0.50, 0.48, 0.51]}
    P1 = {"position": [0.20, -0.40, 0.65], "orientation": [0.71, -0.00, 0.00, 0.70]}
    P2 = {"position": [0.70, -0.23, -0.00], "orientation": [0.56, 0.81, 0.05, 0.10]}
    P3 = {"position": [0.73, 0.42, 0.03], "orientation": [-1.28, 0.99, 0.01, 2.07]}
    D = {"position": [-0.69, 0.10, 0.44], "orientation": [-0.04, 0.99, -0.02, -0.05]}

    # List of points in the desired sequence
    points = [P3, D, P1, D, P3, D]

    # Convert dict to Pose object
    def dict_to_pose(point_dict):
        pose = Pose()
        pose.position = Point(x=point_dict["position"][0], y=point_dict["position"][1], z=point_dict["position"][2])
        pose.orientation = Quaternion(x=point_dict["orientation"][0], y=point_dict["orientation"][1],
                                      z=point_dict["orientation"][2], w=point_dict["orientation"][3])
        return pose

    # Start from the initial position
    start_pose = dict_to_pose(start)

    for point in points:
        end_pose = dict_to_pose(point)
        
        # Interpolate between start and end poses
        intermediate_poses = interpolate_pose(start_pose, end_pose, steps=20)

        # Plan and execute motion through all waypoints
        for pose in intermediate_poses:
            # Set pose goal for each intermediate point
            moveit2.move_to_pose(
                position=[pose.position.x, pose.position.y, pose.position.z],
                quat_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                cartesian=False  # Cartesian path
            )
            traj = moveit2.plan()
            if traj is not None:
                print("Trajectory planned successfully.")
                moveit2.execute(traj)
                moveit2.wait_until_executed()
                print("Trajectory executed.")
                time.sleep(2)
            else:
                print(f"Trajectory planning failed for pose {pose}")

                    # Update the start pose for the next iteration
        start_pose = end_pose

    # Shutdown the node
    rclpy.shutdown()

if __name__ == "__main__":
    main()
