#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""
# #!/usr/bin/env python3
# """
# Example of moving to a pose goal.
# `ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
# """
# #!/usr/bin/env python3
# """
# Example of moving to a pose goal.
# `ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
# """

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
# from rclpy.action import ActionClient

# from control_msgs.action import FollowJointTrajectory


# ...
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.35, 0.10, 0.68])
    node.declare_parameter("quat_xyzw", [0.5, 0.5, 0.5, 0.5])
    node.declare_parameter("cartesian", False)

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

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    start={"position":[0.19, -0.40 , 0.65], "orientation":[3.76, 7.71 , 0.00, 0.99]}
    # Define the coordinates and orientations
       
    P1 = {"position": [0.20, -0.40, 0.65], "orientation": [0.71, -0.00, 0.00, 0.70], "joint_state":[]}

    P2 = {"position": [0.70, -0.23, -0.00], "orientation": [0.56, 0.81, 0.05, 0.10]}

    P3 = {"position": [0.73, 0.42, 0.03], "orientation": [-1.28, 0.99, 0.01, 2.07]}

    D = {"position": [-0.69, 0.10, 0.44], "orientation": [-0.04, 0.99, -0.02, -0.05]}

    intial=[0.0, -2.39, 2.4, -3.15, -1.58, 3.15]

   
    coordinates = [P1, start,D,start, P2,start, P1,start, P2,start,  P1]
    for coordinate in coordinates:
        # moveit2.set_pose_goal(position=coordinate["position"], quat_xyzw=coordinate["orientation"],frame_id="world")
        plan_trajectory=moveit2.plan(position=coordinate["position"], quat_xyzw=coordinate["orientation"],frame_id="world",start_joint_state= intial)
        # moveit2.move_to_configuration()
        moveit2.execute(plan_trajectory)
        # moveit2.move_to_pose(position=coordinate["position"], quat_xyzw=coordinate["orientation"],frame_id="", cartesian=True)
        # moveit2.compute_fk()
        # joint_coordinates = moveit2.compute_ik(position=coordinate["position"], quat_xyzw=coordinate["orientation"])
        # if joint_coordinates is not None:
        #     # Extract the joint positions from the JointState object
        #     joint_positions = joint_coordinates.position
        #     # Move to joint coordinates
        #     moveit2.move_to_configuration(joint_positions)
        # else:
            

        #     print("Failed to compute IK for pose")
            
        moveit2.wait_until_executed()
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()

    

   
   