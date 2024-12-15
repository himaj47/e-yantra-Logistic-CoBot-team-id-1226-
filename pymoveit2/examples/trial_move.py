import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from threading import Thread
import numpy as np
from geometry_msgs.msg import Pose
import time
import random

def create_pose(position, orientation):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
    return pose

def main():
    rclpy.init()

    node = Node("adaptive_path_planning")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    waypoints = [
        create_pose([0.20, -0.40, 0.65], [0.71, -0.00, 0.00, 0.70]),
        create_pose([0.70, -0.23, -0.00], [0.56, 0.81, 0.05, 0.10]),
        create_pose([0.73, 0.42, 0.03], [-1.28, 0.99, 0.01, 2.07]),
        create_pose([-0.69, 0.10, 0.44], [-0.04, 0.99, -0.02, -0.05]),
    ]

    def generate_intermediate_waypoint(start, end):
        # Generate a random intermediate point
        alpha = random.uniform(0.3, 0.7)
        intermediate = Pose()
        intermediate.position.x = start.position.x + alpha * (end.position.x - start.position.x)
        intermediate.position.y = start.position.y + alpha * (end.position.y - start.position.y)
        intermediate.position.z = start.position.z + alpha * (end.position.z - start.position.z)
        
        # Use spherical linear interpolation for orientation
        intermediate.orientation.x = (1 - alpha) * start.orientation.x + alpha * end.orientation.x
        intermediate.orientation.y = (1 - alpha) * start.orientation.y + alpha * end.orientation.y
        intermediate.orientation.z = (1 - alpha) * start.orientation.z + alpha * end.orientation.z
        intermediate.orientation.w = (1 - alpha) * start.orientation.w + alpha * end.orientation.w

        return intermediate

    def attempt_move(start, end, max_attempts=3):
        for attempt in range(max_attempts):
            try:
                if attempt == 0:
                    # First attempt: direct path
                    plan = moveit2.plan(position=end.position, quat_xyzw=[end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w])
                else:
                    # Subsequent attempts: use intermediate waypoint
                    intermediate = generate_intermediate_waypoint(start, end)
                    plan1 = moveit2.plan(position=intermediate.position, quat_xyzw=[intermediate.orientation.x, intermediate.orientation.y, intermediate.orientation.z, intermediate.orientation.w])
                    if plan1 is not None:
                        moveit2.execute(plan1)
                        plan = moveit2.plan(position=end.position, quat_xyzw=[end.orientation.x, end.orientation.y, end.orientation.z, end.orientation.w])
                    else:
                        plan = None

                if plan is not None:
                    success = moveit2.execute(plan)
                    if success:
                        print(f"Successfully moved to waypoint on attempt {attempt + 1}")
                        return True
                    else:
                        print(f"Failed to execute movement on attempt {attempt + 1}")
                else:
                    print(f"Failed to plan movement on attempt {attempt + 1}")
            except Exception as e:
                print(f"An error occurred on attempt {attempt + 1}: {str(e)}")

            time.sleep(1)  # Wait before next attempt

        print(f"Failed to reach waypoint after {max_attempts} attempts.")
        return False

    # current_pose = moveit2.get_current_pose().pose
    # for waypoint in waypoints:
    #     if not attempt_move(current_pose, waypoint):
    #         print("Skipping to next waypoint due to persistent failure.")
    #     current_pose = waypoint

    rclpy.shutdown()

if __name__ == "__main__":
    main()