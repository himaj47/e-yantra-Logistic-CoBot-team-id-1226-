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
import numpy as np
# from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose,Quaternion,Point

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
       
    P1 = {"position": [0.20, -0.40, 0.65], "orientation": [0.71, -0.00, 0.00, 0.70], "joint_state":[np.deg2rad(-77),np.deg2rad(-89),np.deg2rad(81),np.deg2rad(-171),np.deg2rad(-96),np.deg2rad(172)]}

    P2 = {"position": [0.70, -0.23, -0.00], "orientation": [0.56, 0.81, 0.05, 0.10], "joint_state":[np.deg2rad(-26),np.deg2rad(329),np.deg2rad(68),np.deg2rad(-125),np.deg2rad(268),np.deg2rad(144)]}

    P3 = {"position": [0.73, 0.42, 0.03], "orientation": [-1.28, 0.99, 0.01, 2.07], "joint_state":[np.deg2rad(21),np.deg2rad(336),np.deg2rad(57),np.deg2rad(-126),np.deg2rad(269),np.deg2rad(191)]}

    D = {"position": [-0.69, 0.10, 0.44], "orientation": [-0.04, 0.99, -0.02, -0.05], "joint_state":[np.deg2rad(143),np.deg2rad(-66),np.deg2rad(72),np.deg2rad(-100),np.deg2rad(-86),np.deg2rad(-47)]}

    start={"joint_state":[0.0, -2.39, 2.4, -3.15, -1.58, 3.15]}
     
    intial=[0.0, -2.39, 2.4,-3.15,-1.58,3.15]
    
    coordinates = [P3,start,P2 , start, P3,start, D,start ,P3, start, D]
    for coordinate in coordinates:
        # moveit2.set_pose_goal(position=coordinate["position"], quat_xyzw=coordinate["orientation"],frame_id="world")
        # plan_trajectory=moveit2.plan(position=coordinate["position"], quat_xyzw=coordinate["orientation"],frame_id="world",joint_positions=coordinate["joint_state"])
        # moveit2.move_to_configuration()
        # moveit2.execute(plan_trajectory)
        # moveit2.move_to_pose(position=coordinate["position"], quat_xyzw=coordinate["orientation"],frame_id="", cartesian=True)
        Cartesian_state=moveit2.compute_fk(joint_state=coordinate["joint_state"])
        pose=Cartesian_state[0].pose.position
        orienatation=Cartesian_state[0].pose.orientation
        print(pose,orienatation)
        # l=[]
        # l.append(Cartesian_state.pose.position)
        # x=[]
        # x.append(Cartesian_state.pose.orientation)
      
        
        # Pose.position = Cartesian_state[0].
        # Pose.orientation = Cartesian_state.pose.orientation
        # joint_coordinates = moveit2.compute_ik(position=coordinate["position"], quat_xyzw=coordinate["orientation"])
        # moveit2.
        moveit2.set_pose_goal(position=pose,quat_xyzw=orienatation,cartesia)
        traj=moveit2.plan(position=pose,quat_xyzw=orienatation,start_joint_state=intial)
        if traj is not None:

        # Check for collisions

        #  if moveit2.check_collision(traj):

        #     print("Collision detected, trying again...")

        #     # Try to plan a new trajectory with a different set of joint angles

        #     coordinate["joint_state"] = [x + 0.1 for x in coordinate["joint_state"]]

        #  else:

            # Execute the trajectory

            moveit2.execute(traj)

            moveit2.wait_until_executed()

            

        else:

          print("Invalid trajectory, trying again...")

        # Try to plan a new trajectory with a different set of joint angles

          coordinate["joint_state"] = [x + 0.1 for x in coordinate["joint_state"]]
        # moveit2.move_to_configuration()
        # print(type(Cartesian_state), Cartesian_state)
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

    

   
   