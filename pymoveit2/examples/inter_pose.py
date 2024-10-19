import numpy as np
from geometry_msgs.msg import Pose, Quaternion, Point
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
import rclpy
from rclpy.node import Node 
def linear_interpolation(start, end, steps):
    """Linear interpolation between start and end point"""
    return [start + (end - start) * t for t in np.linspace(0, 1, steps)]
from scipy.spatial.transform import Slerp, Rotation as R

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

    # Define start and end poses
    start_pose = Pose()
    start_pose.position = Point(x=0.19, y=-0.40, z=0.65)
    start_pose.orientation = Quaternion(x=3.76, y=7.71, z=0.00, w=0.99)

    end_pose = Pose()
    end_pose.position = Point(x=0.22, y=-0.44, z=0.63)
    end_pose.orientation =Quaternion(x=0.70, y=0.07, z=0.07, w=0.69)

    # Generate intermediate points between start and end poses
    intermediate_poses = interpolate_pose(start_pose, end_pose, steps=20)

    # Plan and execute motion through all waypoints
    for pose in intermediate_poses:
        # Set pose goal for each intermediate point
        moveit2.move_to_pose(
            position=[pose.position.x, pose.position.y, pose.position.z],
            quat_xyzw=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            cartesian=False # Cartesian path
        )
        # moveit2.move_to_pose
        # Plan the trajectory
        traj = moveit2.plan()
        if traj is not None:
            # Execute the planned trajectory
            moveit2.execute(traj)
            moveit2.wait_until_executed()
        else:
            print("Trajectory planning failed for one of the intermediate poses")

    # Shutdown the node
    rclpy.shutdown()

if __name__ == "__main__":
    main()
