#!/usr/bin/env python3

# # Import necessary ROS2 packages and message types
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Range
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
# from tf_transformations import euler_from_quaternion
# from ebot_docking.srv import DockSw  # Import custom service message
# import math
# class MyRobotDockingController(Node):

#     def __init__(self):
#         super().__init__('my_robot_docking_controller')
#         self.callback_group = ReentrantCallbackGroup()

#         # Initialize target distance and alignment status
#         self.target_distance = 0.5  # Default target distance in meters
#         self.is_docking = False
#         self.dock_aligned = False
#         self.docking_in_progress = False  # Flag to track if docking is already in progress

#         # Subscriptions and service setup
#         self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
#         self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
#         self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)
#         self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         self.controller_timer = self.create_timer(0.1, self.controller_loop)

#     # Callback for odometry data
#     def odometry_callback(self, msg):
#         self.robot_pose = [
#             msg.pose.pose.position.x,
#             msg.pose.pose.position.y,
#             euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
#                                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]
#         ]

#     # Callback functions for ultrasonic sensors
#     def ultrasonic_rl_callback(self, msg):
#         self.usrleft_value = msg.range

#     def ultrasonic_rr_callback(self, msg):
#         self.usrright_value = msg.range

#     # Main control loop for docking behavior
#     def controller_loop(self):
#         if self.is_docking and not self.docking_in_progress:
#             self.docking_in_progress = True  # Mark docking in progress
#             self.get_logger().info(f"Docking started! Target distance: {self.target_distance} meters")

#         if self.is_docking:
#             kp_linear = 0.5  # Adjust this for better docking precision
#             kp_angular = 0.5  # Adjust this for better alignment

#             # Calculate error for distance and alignment
#             distance_error = min(self.usrleft_value, self.usrright_value) - self.target_distance
#             alignment_error = self.usrleft_value - self.usrright_value

#             # Set linear and angular velocities based on error
#             linear_vel = kp_linear * distance_error
#             angular_vel = kp_angular * alignment_error

#             # Publish velocities for docking
#             twist_msg = Twist()
#             twist_msg.linear.x = linear_vel
#             twist_msg.angular.z = angular_vel
#             self.cmd_vel_pub.publish(twist_msg)

#             # Check if we're within the alignment threshold to stop docking
#             if abs(distance_error) < 0.05 and abs(alignment_error) < 0.05:
#                 self.dock_aligned = True
#                 self.is_docking = False
#                 self.docking_in_progress = False  # Docking completed

#                 self.get_logger().info("Docking successful and alignment achieved!")

#     # Docking service callback to initiate docking and set target distance
#     def dock_control_callback(self, request, response):
#         if self.is_docking:
#             # If docking is already in progress, prevent re-triggering
#             response.success = False
#             response.message = "Docking is already in progress."
#             self.get_logger().warn("Docking already in progress, cannot initiate again.")
#             return response

#         # Set the target distance from the request if provided
#         self.target_distance = request.distance if request.distance else self.target_distance
#         self.is_docking = True
#         self.dock_aligned = False

#         self.get_logger().info("Docking started!")
#         response.success = True
#         response.message = "Docking control initiated"
#         return response

# def main(args=None):
#     rclpy.init(args=args)
#     my_robot_docking_controller = MyRobotDockingController()
#     executor = MultiThreadedExecutor()
#     executor.add_node(my_robot_docking_controller)
#     executor.spin()

#     my_robot_docking_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
#!/usr/bin/env python3

## Overview
# ###
# # This ROS2 script is designed to control a robot's docking behavior with a rack.
# # It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service.
# # The script handles both linear and angular motion to achieve docking alignment and execution.
# #### Import necessary ROS2 packages and message types
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Range
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
# from tf_transformations import euler_from_quaternion
# from ebot_docking.srv import DockSw  # Import custom service message
# import math

# # Define a class for your ROS2 node
# class MyRobotDockingController(Node):

#     def __init__(self):
#         # Initialize the ROS2 node with a unique name
#         super().__init__('my_robot_docking_controller')

#         # Create a callback group for managing callbacks
#         self.callback_group = ReentrantCallbackGroup()
#         self.previous_dis=0.0
#         # Subscribe to odometry data for robot pose information
#         self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

#         # Subscribe to ultrasonic sensor data for distance measurements
#         self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
#         self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

#         # Create a ROS2 service for controlling docking behavior
#         self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback, callback_group=self.callback_group)

#         # Create a publisher for sending velocity commands to the robot
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Initialize flags and parameters
#         self.is_docking = False
#         self.docking_in_progress = False
#         self.dock_aligned = False
#         self.docking_complete = False  # New flag to track docking completion
#         self.target_distance = 0.08  # Target distance to the docking object
#         self.robot_pose = [0.0, 0.0, 0.0]
#         self.usrleft_value = float('inf')  # Initialize sensor readings
#         self.usrright_value = float('inf')

#         # Initialize a timer for the main control loop
#         self.controller_timer = self.create_timer(0.1, self.controller_loop)

#     # Callback function for odometry data
#     def odometry_callback(self, msg):
#         # Extract and update robot pose information from odometry message
#         self.robot_pose[0] = msg.pose.pose.position.x
#         self.robot_pose[1] = msg.pose.pose.position.y
#         quaternion_array = msg.pose.pose.orientation
#         orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
#         _, _, yaw = euler_from_quaternion(orientation_list)
#         self.robot_pose[2] = yaw

#     # Callback function for the left ultrasonic sensor
#     def ultrasonic_rl_callback(self, msg):
#         self.usrleft_value = msg.range

#     # Callback function for the right ultrasonic sensor
#     def ultrasonic_rr_callback(self, msg):
#         self.usrright_value = msg.range

#     def controller_loop(self):
#         if not self.is_docking:
#             return

#         kp_linear = 2.1
#         kp_angular = 1.8
#         max_linear_vel = 0.5
#         max_angular_vel = 0.5

#         # distance_error = (self.usrleft_value + self.usrright_value) / 2
#         distance_error = min(self.usrleft_value , self.usrright_value) 
#         delta_distance = distance_error - self.previous_dis
#         self.previous_dis = distance_error

#         if distance_error > self.target_distance:
#             linear_vel = kp_linear * distance_error - 0.1 * delta_distance
#             angular_vel = 0.0
#             self.get_logger().info(f"Approaching target, distance error: {distance_error}")
#         else:
#             alignment_error = self.usrleft_value - self.usrright_value
#             if abs(alignment_error) > 0.10:
#                 angular_vel = kp_angular * alignment_error
#                 linear_vel = 0.0
#                 self.get_logger().info(f"Aligning with error: {alignment_error}")
#             else:
#                 linear_vel = 0.0
#                 angular_vel = 0.0
#                 # if abs(linear_vel) < 0.01 and abs(angular_vel) < 0.01:  # Ensure stationary
#                 self.is_docking = False
#                 self.docking_complete = True
                
#                 self.get_logger().info("Docking complete!")
#                 self.on_docking_complete()

#         # linear_vel = min(max_linear_vel, max(-max_linear_vel, linear_vel))
#         # angular_vel = min(max_angular_vel, max(-max_angular_vel, angular_vel))

#         twist_msg = Twist()
#         twist_msg.linear.x = -linear_vel
#         twist_msg.angular.z = -angular_vel
#         self.cmd_vel_pub.publish(twist_msg)


#     # Callback function for the DockControl service
#     # def dock_control_callback(self, request, response):
#     #     if self.docking_complete:
#     #         response.success = True
#     #         response.message = "Docking is already in progress."
#     #         return response

#     #     # Start the docking process
#     #     self.is_docking = True
#     #     self.docking_complete = False  # Reset completion flag for a new docking cycle
#     #     response.success = False
#     #     response.message = "Docking initiated"
#     #     self.get_logger().info("Starting docking process")
#     #     return response
#     # Callback function for the DockControl service
#     def dock_control_callback(self, request, response):
#         if self.docking_complete:
#             # If docking is complete, allow a new request to be processed
#             self.is_docking = True  # Re-initiate docking
#             self.docking_complete = False  # Reset docking completion flag
#             response.success = True
#             response.message = "Docking completed. New docking request initiated."
#             self.get_logger().info("Docking completed. Processing new request.")
#             return response

#         # If docking is not complete, block new requests
#         if self.docking_in_progress:
#             response.success = False
#             response.message = "Docking is in progress. Please wait until it's completed."
#             self.get_logger().info("Docking in progress. Blocking new request.")
#             return response

#         # Start the docking process
#         self.docking_in_progress = True  # Flag to indicate docking is in progress
#         self.is_docking = True  # Initiating docking action
#         self.docking_complete = False  # Ensure completion flag is reset
#         response.success = False
#         response.message = "Docking initiated"
#         self.get_logger().info("Starting docking process")
#         return response


    

#     # Handle actions to perform when docking is completed
#     def on_docking_complete(self):
#         self.get_logger().info("Docking process completed successfully")
#         # Add any additional logic here, such as resuming navigation, resetting flags, etc.

#     # Function to check if docking is complete (to be used externally if needed)
#     def is_docking_completed(self):
#         return self.docking_complete

# # Main function to initialize the ROS2 node and spin the executor
# def main(args=None):
#     rclpy.init(args=args)

#     my_robot_docking_controller = MyRobotDockingController()

#     executor = MultiThreadedExecutor()
#     executor.add_node(my_robot_docking_controller)

#     executor.spin()

#     my_robot_docking_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()










# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Range
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
# from tf_transformations import euler_from_quaternion
# from ebot_docking.srv import DockSw  # Import custom service message
# import math

# class MyRobotDockingController(Node):

#     def __init__(self):
#         super().__init__('my_robot_docking_controller')

#         # Create a callback group for managing callbacks
#         self.callback_group = ReentrantCallbackGroup()
#         self.previous_dis = 0.0

#         # Subscribe to odometry data for robot pose information
#         self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

#         # Subscribe to ultrasonic sensor data for distance measurements
#         self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
#         self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

#         # Create a ROS2 service for controlling docking behavior
#         self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback, callback_group=self.callback_group)

#         # Create a publisher for sending velocity commands to the robot
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Initialize flags and parameters
#         self.is_docking = False
#         self.docking_in_progress = False
#         self.dock_aligned = False
#         self.docking_complete = False  # New flag to track docking completion
#         self.target_distance = 0.08  # Target distance to the docking object
#         self.robot_pose = [0.0, 0.0, 0.0]
#         self.usrleft_value = float('inf')  # Initialize sensor readings
#         self.usrright_value = float('inf')

#         # Initialize a timer for the main control loop
#         self.controller_timer = self.create_timer(0.1, self.controller_loop)

#     # Callback function for odometry data
#     def odometry_callback(self, msg):
#         # Extract and update robot pose information from odometry message
#         self.robot_pose[0] = msg.pose.pose.position.x
#         self.robot_pose[1] = msg.pose.pose.position.y
#         quaternion_array = msg.pose.pose.orientation
#         orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
#         _, _, yaw = euler_from_quaternion(orientation_list)
#         self.robot_pose[2] = yaw

#     # Callback function for the left ultrasonic sensor
#     def ultrasonic_rl_callback(self, msg):
#         self.usrleft_value = msg.range

#     # Callback function for the right ultrasonic sensor
#     def ultrasonic_rr_callback(self, msg):
#         self.usrright_value = msg.range

#     def controller_loop(self):
#         if not self.is_docking:
#             return

#         kp_linear = 2.1
#         kp_angular = 1.8
#         max_linear_vel = 0.5
#         max_angular_vel = 0.5

#         distance_error = min(self.usrleft_value, self.usrright_value)
#         delta_distance = distance_error - self.previous_dis
#         self.previous_dis = distance_error

#         if distance_error > self.target_distance:
#             linear_vel = kp_linear * distance_error - 0.1 * delta_distance
#             angular_vel = 0.0
#             self.get_logger().info(f"Approaching target, distance error: {distance_error}")
#         else:
#             alignment_error = self.usrleft_value - self.usrright_value
#             if abs(alignment_error) > 0.10:
#                 angular_vel = kp_angular * alignment_error
#                 linear_vel = 0.0
#                 self.get_logger().info(f"Aligning with error: {alignment_error}")
#             else:
#                 linear_vel = 0.0
#                 angular_vel = 0.0
#                 self.is_docking = False
#                 self.docking_complete = True
                
#                 self.get_logger().info("Docking complete!")
#                 self.on_docking_complete()

#         twist_msg = Twist()
#         twist_msg.linear.x = -linear_vel
#         twist_msg.angular.z = -angular_vel
#         self.cmd_vel_pub.publish(twist_msg)

#     def dock_control_callback(self, request, response):
#         if self.docking_complete:
#             # If docking is complete, allow a new request to be processed
#             self.is_docking = True  # Re-initiate docking
#             self.docking_complete = False  # Reset docking completion flag
#             response.success = True
#             response.message = "Docking completed. New docking request initiated."
#             self.get_logger().info("Docking completed. Processing new request.")
#             return response

#         # If docking is not complete, block new requests
#         if self.docking_in_progress:
#             response.success = False
#             response.message = "Docking is in progress. Please wait until it's completed."
#             self.get_logger().info("Docking in progress. Blocking new request.")
#             return response

#         # Start the docking process
#         self.docking_in_progress = True  # Flag to indicate docking is in progress
#         self.is_docking = True  # Initiating docking action
#         self.docking_complete = False  # Ensure completion flag is reset
#         response.success = False
#         response.message = "Docking initiated"
#         self.get_logger().info("Starting docking process")
#         return response
    

#     def on_docking_complete(self):
#         self.get_logger().info("Docking process completed successfully")
#         # Add any additional logic here, such as resuming navigation, resetting flags, etc.
#         self.docking_in_progress = False  # Allow new requests to be processed
#         self.docking_complete = True  # Ensure completion flag is set
#         # Additional logic can be added here after docking is complete

#     def is_docking_completed(self):
#         return self.docking_complete

# # Main function to initialize the ROS2 node and spin the executor
# def main(args=None):
#     rclpy.init(args=args)

#     my_robot_docking_controller = MyRobotDockingController()

#     executor = MultiThreadedExecutor()
#     executor.add_node(my_robot_docking_controller)

#     executor.spin()

#     my_robot_docking_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Range
# from tf_transformations import euler_from_quaternion
# from ebot_docking.srv import DockSw  # Import custom service message
# import math
# import time

# class MyRobotDockingController(Node):

#     def __init__(self):
#         super().__init__('my_robot_docking_controller')

#         # Subscribers for odometry and ultrasonic sensor data
#         self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
#         self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
#         self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

#         # Service for docking control
#         self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback)

#         # Publisher for robot velocity commands
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # State variables for docking
#         self.is_docking = False
#         self.docking_complete = False
#         self.target_distance = 0.08  # Target distance to the docking object
#         self.robot_pose = [0.0, 0.0, 0.0]
#         self.usrleft_value = float('inf')  # Initialize sensor readings
#         self.usrright_value = float('inf')

#     def odometry_callback(self, msg):
#         """Update robot pose from odometry data."""
#         self.robot_pose[0] = msg.pose.pose.position.x
#         self.robot_pose[1] = msg.pose.pose.position.y
#         quaternion_array = msg.pose.pose.orientation
#         orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
#         _, _, yaw = euler_from_quaternion(orientation_list)
#         self.robot_pose[2] = yaw

#     def ultrasonic_rl_callback(self, msg):
#         """Update left ultrasonic sensor data."""
#         self.usrleft_value = msg.range

#     def ultrasonic_rr_callback(self, msg):
#         """Update right ultrasonic sensor data."""
#         self.usrright_value = msg.range

#     def dock_control_callback(self, request, response):
#         """Service callback to handle docking requests."""
#         if self.is_docking:
#             response.success = False
#             response.message = "Docking already in progress."
#             self.get_logger().info("Docking request received but already in progress.")
#             return response

#         # Start docking process
#         self.is_docking = True
#         self.docking_complete = False
#         self.get_logger().info("Docking process initiated.")
        
#         # Perform the docking process
#         while not self.docking_complete:
#             success = self.perform_docking()
#             if success:
#                 self.docking_complete = True
#             else:
#                 self.docking_complete = False
#                 self.get_logger().info("Trying to reach the target...")
#                 # time.sleep(0.1)  # Allow other processes to run

#         # Docking process complete
#         response.success = True
#         response.message = "Docking complete."
#         return response

#     def perform_docking(self):
#         """Main docking logic to control the robot until it reaches the target."""
#         kp_linear = 2.1
#         kp_angular = 1.8
#         max_linear_vel = 0.5
#         max_angular_vel = 0.5

#         distance_error = min(self.usrleft_value, self.usrright_value)

#         if distance_error > self.target_distance:
#             # Move towards the target
#             linear_vel = kp_linear * distance_error
#             angular_vel = 0.0
#             self.get_logger().info(f"Approaching target, distance error: {distance_error}")
#         else:
#             # Align with the target when close enough
#             alignment_error = self.usrleft_value - self.usrright_value
#             if abs(alignment_error) > 0.10:
#                 angular_vel = kp_angular * alignment_error
#                 linear_vel = 0.0
#                 self.get_logger().info(f"Aligning with error: {alignment_error}")
#             else:
#                 # Docking is considered complete
#                 self.is_docking = False
#                 return True

#         # Publish velocity commands to move the robot
#         twist_msg = Twist()
#         twist_msg.linear.x = -min(max(linear_vel, -max_linear_vel), max_linear_vel)
#         twist_msg.angular.z = -min(max(angular_vel, -max_angular_vel), max_angular_vel)
#         self.cmd_vel_pub.publish(twist_msg)

#         return False  # Return False to indicate docking is still in progress

# # Main function to initialize and run the node
# def main(args=None):
#     rclpy.init(args=args)

#     my_robot_docking_controller = MyRobotDockingController()

#     rclpy.spin(my_robot_docking_controller)

#     my_robot_docking_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Range
# from tf_transformations import euler_from_quaternion
# from ebot_docking.srv import DockSw  # Import custom service message
# import math


# class MyRobotDockingController(Node):

#     def __init__(self):
#         super().__init__('my_robot_docking_controller')

#         # Subscribers for odometry and ultrasonic sensor data
#         self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
#         self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
#         self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

#         # Service for docking control
#         self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback)

#         # Publisher for robot velocity commands
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # State variables for docking
#         self.is_docking = False
#         self.docking_complete = False
#         self.target_distance = 0.08  # Target distance to the docking object
#         self.robot_pose = [0.0, 0.0, 0.0]
#         self.usrleft_value = float('inf')  # Initialize sensor readings
#         self.usrright_value = float('inf')

#         # Timer for docking process (will be created during docking)
#         self.docking_timer = None
#         self.docking_future = None  # Added for async handling

#     def odometry_callback(self, msg):
#         """Update robot pose from odometry data."""
#         self.robot_pose[0] = msg.pose.pose.position.x
#         self.robot_pose[1] = msg.pose.pose.position.y
#         quaternion_array = msg.pose.pose.orientation
#         orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
#         _, _, yaw = euler_from_quaternion(orientation_list)
#         self.robot_pose[2] = yaw

#     def ultrasonic_rl_callback(self, msg):
#         """Update left ultrasonic sensor data."""
#         self.usrleft_value = msg.range
#         self.get_logger().info(f"Updated left ultrasonic sensor value: {self.usrleft_value}")

#     def ultrasonic_rr_callback(self, msg):
#         """Update right ultrasonic sensor data."""
#         self.usrright_value = msg.range
#         self.get_logger().info(f"Updated right ultrasonic sensor value: {self.usrright_value}")

#     def dock_control_callback(self, request, response):
#         """Service callback to handle docking requests."""
#         if self.is_docking:
#             response.success = False
#             response.message = "Docking already in progress."
#             self.get_logger().info("Docking request received but already in progress.")
#             return response

#         if self.docking_complete:
#             response.success = True
#             response.message = "Docking has already been completed."
#             self.get_logger().info("Docking request received but already completed.")
#             return response

#         # Start docking process
#         self.is_docking = True
#         self.docking_complete = False
#         self.get_logger().info("Docking process initiated.")
        
#         # Create a Future object to monitor the completion asynchronously
#         self.docking_future = self.create_future()
#         self.docking_timer = self.create_timer(0.1, self.perform_docking_timer_callback)
        
#         # Non-blocking handling for service response
#         self.docking_future.add_done_callback(lambda future: self.complete_docking(future, response))
#         return response

#     def perform_docking_timer_callback(self):
#         """Non-blocking timer-based docking logic."""
#         success = self.perform_docking()
#         if success:
#             self.docking_complete = True
#             self.is_docking = False
#             if self.docking_timer:
#                 self.docking_timer.cancel()  # Stop the timer once docking is complete
#             self.get_logger().info("Docking complete.")
#             self.docking_future.set_result(True)

#     def perform_docking(self):
#         """Main docking logic to control the robot until it reaches the target."""
#         kp_linear = 2.1
#         kp_angular = 1.8
#         max_linear_vel = 0.5
#         max_angular_vel = 0.5

#         distance_error = min(self.usrleft_value, self.usrright_value)
#         self.get_logger().info(f"Current distance error: {distance_error}")

#         if distance_error > self.target_distance:
#             # Move towards the target
#             linear_vel = kp_linear * distance_error
#             angular_vel = 0.0
#             self.get_logger().info(f"Approaching target, distance error: {distance_error}")
#         else:
#             # Align with the target when close enough
#             alignment_error = self.usrleft_value - self.usrright_value
#             if abs(alignment_error) > 0.10:
#                 angular_vel = kp_angular * alignment_error
#                 linear_vel = 0.0
#                 self.get_logger().info(f"Aligning with error: {alignment_error}")
#             else:
#                 # Docking is considered complete
#                 self.is_docking = False
#                 return True

#         # Publish velocity commands to move the robot
#         twist_msg = Twist()
#         twist_msg.linear.x = -linear_vel
#         twist_msg.angular.z = -angular_vel
#         self.cmd_vel_pub.publish(twist_msg)

#         return False  # Return False to indicate docking is still in progress

#     def complete_docking(self, future, response):
#         """Completes the docking service request asynchronously."""
#         response.success = True
#         response.message = "Docking process completed successfully."
#         self.get_logger().info("Docking service request completed.")


# # Main function to initialize and run the node
# def main(args=None):
#     rclpy.init(args=args)

#     my_robot_docking_controller = MyRobotDockingController()

#     rclpy.spin(my_robot_docking_controller)

#     my_robot_docking_controller.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Range
# from tf_transformations import euler_from_quaternion
# from ebot_docking.srv import DockSw  # Import custom service message
# from payload_service.srv import PayloadSW  # Import payload drop service
# import math


# class MyRobotDockingController(Node):

#     def __init__(self):
#         super().__init__('my_robot_docking_controller')

#         # Subscribers for odometry and ultrasonic sensor data
#         self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
#         self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
#         self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

#         # Service for docking control
#         self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback)

#         # Publisher for robot velocity commands
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Payload drop client
#         self.payload_client = self.create_client(PayloadSW, '/payload_sw')
#         while not self.payload_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for PayloadSW service...')

#         # State variables for docking
#         self.is_docking = False
#         self.docking_complete = False
#         self.target_distance = 0.10  # Target distance to the docking object
#         self.robot_pose = [0.0, 0.0, 0.0]
#         self.usrleft_value = float('inf')  # Initialize sensor readings
#         self.usrright_value = float('inf')

#         # Timer for docking process (will be created during docking)
#         self.docking_timer = None

#     def odometry_callback(self, msg):
#         """Update robot pose from odometry data."""
#         self.robot_pose[0] = msg.pose.pose.position.x
#         self.robot_pose[1] = msg.pose.pose.position.y
#         quaternion_array = msg.pose.pose.orientation
#         orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
#         _, _, yaw = euler_from_quaternion(orientation_list)
#         self.robot_pose[2] = yaw

#     def ultrasonic_rl_callback(self, msg):
#         """Update left ultrasonic sensor data."""
#         self.usrleft_value = msg.range
#         # self.get_logger().info(f"Updated left ultrasonic sensor value: {self.usrleft_value}")

#     def ultrasonic_rr_callback(self, msg):
#         """Update right ultrasonic sensor data."""
#         self.usrright_value = msg.range
#         # self.get_logger().info(f"Updated right ultrasonic sensor value: {self.usrright_value}")

#     def dock_control_callback(self, request, response):
#         """Service callback to handle docking requests."""
#         if self.is_docking:
#             response.success = False
#             response.message = "Docking already in progress."
#             self.get_logger().info("Docking request received but already in progress.")
#             return response

#         if self.docking_complete:
#             response.success = True
#             response.message = "Docking has already been completed."
#             self.get_logger().info("Docking request received but already completed.")
#             return response

#         # Start docking process
#         self.is_docking = True
#         self.docking_complete = False
#         self.get_logger().info("Docking process initiated.")
        
#         # Create a timer to handle the docking process non-blockingly
#         self.docking_timer = self.create_timer(0.1, self.perform_docking_timer_callback)
        
#         # Respond immediately to indicate the docking process has started
#         response.success = True
#         response.message = "Docking process started."
#         return response

#     def perform_docking_timer_callback(self):
#         """Non-blocking timer-based docking logic."""
#         success = self.perform_docking()
#         if success:
#             self.docking_complete = True
#             self.is_docking = False
#             if self.docking_timer:
#                 self.docking_timer.cancel()  # Stop the timer once docking is complete
#             self.get_logger().info("Docking complete. Initiating payload drop.")
#             self.trigger_payload_drop()

#     def perform_docking(self):
#         """Main docking logic to control the robot until it reaches the target."""
#         kp_linear = 2.1
#         kp_angular = 1.8
#         max_linear_vel = 0.5
#         max_angular_vel = 0.5

#         distance_error = min(self.usrleft_value, self.usrright_value)
#         self.get_logger().info(f"Current distance error: {distance_error}")

#         if distance_error > self.target_distance:
#             # Move towards the target
#             linear_vel = kp_linear * distance_error
#             angular_vel = 0.0
#             self.get_logger().info(f"Approaching target, distance error: {distance_error}")
#         else:
#             # Align with the target when close enough
#             alignment_error = self.usrleft_value - self.usrright_value
#             if abs(alignment_error) > 0.20:
#                 angular_vel = kp_angular * alignment_error
#                 linear_vel = 0.0
#                 self.get_logger().info(f"Aligning with error: {alignment_error}")
#             else:
#                 # Docking is considered complete
#                 self.is_docking = False
#                 return True

#         # Publish velocity commands to move the robot
#         twist_msg = Twist()
#         twist_msg.linear.x = -linear_vel
#         twist_msg.angular.z = -angular_vel
#         self.cmd_vel_pub.publish(twist_msg)

#         return False  # Return False to indicate docking is still in progress

#     def trigger_payload_drop(self):
#         """Send a request to trigger the payload drop."""
#         request = PayloadSW.Request()
#         request.receive = False
#         request.drop = True
        
#         self.get_logger().info(f"docking reuest send: {request}")
#         future = self.payload_client.call_async(request)
#         future.add_done_callback(self.payload_drop_response_callback)

#     def payload_drop_response_callback(self, future):
#         """Callback to handle the response from the payload drop service."""
#         try:
#             response = future.result()
#             if response.success:
#                 self.get_logger().info(f"Payload dropped successfully: {response.message}")
#             else:
#                 self.get_logger().warn(f"Payload drop failed: {response.message}")
#         except Exception as e:
#             self.get_logger().error(f"Service call failed: {e}")


# # Main function to initialize and run the node
# def main(args=None):
#     rclpy.init(args=args)

#     my_robot_docking_controller = MyRobotDockingController()

#     rclpy.spin(my_robot_docking_controller)

#     my_robot_docking_controller.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



# below is working code for docking 

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Range
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
# from tf_transformations import euler_from_quaternion
# from ebot_docking.srv import DockSw  # Import custom service message
# import math

# class MyRobotDockingController(Node):

#     def __init__(self):
#         super().__init__('my_robot_docking_controller')

#         # Create a callback group for managing callbacks
#         self.callback_group = ReentrantCallbackGroup()
#         self.previous_dis = 0.0

#         # Subscribe to odometry data for robot pose information
#         self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

#         # Subscribe to ultrasonic sensor data for distance measurements
#         self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
#         self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

#         # Create a ROS2 service for controlling docking behavior
#         self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback, callback_group=self.callback_group)

#         # Create a publisher for sending velocity commands to the robot
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

#         # Initialized flags and parameters
#         self.is_docking = False
#         self.docking_complete = False  # New flag to track docking completion
#         self.robot_pose = [0.0, 0.0, 0.0]
#         self.usrleft_value = float('inf')  # Initialize sensor readings
#         self.usrright_value = float('inf')

#         # Initialize a timer for the main control loop
#         self.controller_timer = self.create_timer(0.1, self.controller_loop)

#     # Callback function for odometry data
#     def odometry_callback(self, msg):
#         # Extract and update robot pose information from odometry message
#         self.robot_pose[0] = msg.pose.pose.position.x
#         self.robot_pose[1] = msg.pose.pose.position.y
#         quaternion_array = msg.pose.pose.orientation
#         orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
#         _, _, yaw = euler_from_quaternion(orientation_list)
#         self.robot_pose[2] = yaw

#     # Callback function for the left ultrasonic sensor
#     def ultrasonic_rl_callback(self, msg):
#         self.usrleft_value = msg.range

#     # Callback function for the right ultrasonic sensor
#     def ultrasonic_rr_callback(self, msg):
#         self.usrright_value = msg.range
    
#     #Function to Calculate Angular Error
#     def  calculate_angular_error(self,Kp_angular,min_orient_error):
#         alignment_error = self.usrleft_value - self.usrright_value
#         if abs(alignment_error) > min_orient_error:
#             angular_vel = Kp_angular * alignment_error
#             self.get_logger().info(f"Approaching target, angualr error: {alignment_error}")   
#         else:
#             angular_vel=0.0
                
#         return angular_vel
    
#      #Function to Calculate linear Error
#     def calculate_linear_error(self,Kp_linear,safe_distance):
#         # distance_error = (self.usrleft_value+self.usrright_value)/2
#         distance_error = min(self.usrleft_value,self.usrright_value)
       
#         delta_distance = distance_error - self.previous_dis
#         self.previous_dis = distance_error

#         if distance_error > safe_distance:
#             linear_vel = Kp_linear * distance_error - 0.1 * delta_distance
#             self.get_logger().info(f"Approaching target, distance error: {distance_error}")
#         else:
#             linear_vel=0.0
#             self.docking_complete=True
#             self.get_logger().info(f'Aligned Successfully')
#             self.is_docking=False
#         return linear_vel

#     def controller_loop(self):
#         if not self.is_docking:
#             return
#         safe_distance=0.05
#         min_orient_error=0.01 #0.01
#         kp_linear = 1.0    #Proportional for linear
#         kp_angular = 2.975    #Proportional for angular
#         max_linear_vel = 0.6
#         max_angular_vel = 0.6
#         # self.linear_error=0.0
#         # Get the angular error
#         angualr_error=self.calculate_angular_error(kp_angular,min_orient_error)
#         # if abs(angualr_error)<=min_orient_error:
#           #Get the linear error
#         self.linear_error=self.calculate_linear_error(kp_linear,safe_distance)
       
#         twist_msg = Twist()
#         twist_msg.linear.x = -min(self.linear_error,max_linear_vel)
#         twist_msg.angular.z = -min(angualr_error,max_angular_vel)
#         self.cmd_vel_pub.publish(twist_msg)

#     def dock_control_callback(self, request, response):
       
#         rate=self.create_rate(100,self.get_clock())
#         # Start the docking process  
#         self.is_docking = True  # Initiating docking action
#         self.docking_complete = False  #  completion flag is reset to false

#         while not self.docking_complete:
#             self.get_logger().info("Waiting for docking to complete")
#             rate.sleep()

#         response.success = True
#         response.message = "Docking completed"
        
#         return response

# # Main function to initialize the ROS2 node and spin the executor
# def main(args=None):
#     rclpy.init(args=args)

#     my_robot_docking_controller = MyRobotDockingController()

#     executor = MultiThreadedExecutor()
#     executor.add_node(my_robot_docking_controller)

#     executor.spin()

#     my_robot_docking_controller.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()





#checking with yaw of the robot


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math

class MyRobotDockingController(Node):

    def __init__(self):
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()
        self.previous_dis = 0.0
        self.prev_angular_error = 0.0
        self.is_angular_aligned = False  # Initialize angular alignment flag
        self.desired_yaw = 0.0  # Placeholder for target yaw

        # Subscribe to odometry and sensor data
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Create a service for docking control
        self.dock_control_srv = self.create_service(DockSw, '/dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Internal state variables
        self.is_docking = False
        self.docking_complete = False
        self.robot_pose = [0.0, 0.0, 0.0]  # [x, y, yaw]
        self.usrleft_value = float('inf')  # Initialize sensor readings
        self.usrright_value = float('inf')

        # Timer for the control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    def odometry_callback(self, msg):
        # Update robot pose from odometry data
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range

    def calculate_angular_error(self, Kp_angular, Kd_angular, desired_yaw, min_orient_error, prev_angular_error):
        # Calculate angular error and normalize it
        angular_error = (desired_yaw - self.robot_pose[2])-3.24  
        angular_error = math.atan2(math.sin(angular_error), math.cos(angular_error))  # Normalize to [-pi, pi]

        # Calculate derivative of the angular error
        angular_error_derivative = angular_error - prev_angular_error

        if abs(angular_error) > min_orient_error:
            # PD control
            angular_vel = Kp_angular * angular_error + Kd_angular * angular_error_derivative
            self.get_logger().info(f"Angular error (normalized): {angular_error} | Derivative: {angular_error_derivative}")
        else:
            angular_vel = 0.0
            self.is_angular_aligned = True  # Mark angular alignment as complete

        return angular_vel, angular_error

    def calculate_linear_error(self, Kp_linear, safe_distance):
        # Calculate distance error from ultrasonic sensors
        distance_error = (self.usrleft_value + self.usrright_value) / 2
        delta_distance = distance_error - self.previous_dis
        self.previous_dis = distance_error

        if distance_error > safe_distance:
            linear_vel = Kp_linear * distance_error - 0.1 * delta_distance
            self.get_logger().info(f"Distance error: {distance_error}")
        else:
            linear_vel = 0.0
            self.docking_complete = True
            self.get_logger().info("Docking complete.")
            self.is_docking = False

        return linear_vel

    def controller_loop(self):
        if not self.is_docking:
            return

        # Control parameters
        safe_distance = 0.06
        min_orient_error = 0.05  # Adjusted for finer angular precision
        kp_linear = 1.9  # Proportional gain for linear control
        kp_angular = 1.6  # Proportional gain for angular control
        kd_angular = 0.0001  # Derivative gain for angular control
        max_linear_vel = 0.8
        max_angular_vel = 0.8

        # Angular alignment logic
        if not self.is_angular_aligned:
            # Calculate angular error using yaw from odometry if not already aligned
            angular_vel, angular_error = self.calculate_angular_error(
                kp_angular, kd_angular, self.desired_yaw, min_orient_error, self.prev_angular_error
            )
            # Store the current angular error for the next loop iteration
            self.prev_angular_error = angular_error

            # Generate and publish only angular velocity if not aligned
            twist_msg = Twist()
            twist_msg.angular.z = min(angular_vel, max_angular_vel)
            twist_msg.linear.x = 0.0  # Prevent linear movement until angular alignment is done
            self.cmd_vel_pub.publish(twist_msg)
            
            # Return early to avoid engaging linear control
            return

        # Linear alignment logic - Only reached if angular alignment is complete
        angular_vel = 0.0  # Stop angular correction once aligned
        self.linear_error = self.calculate_linear_error(kp_linear, safe_distance)

        # Generate and publish velocity commands for linear movement
        twist_msg = Twist()
        twist_msg.angular.z = angular_vel
        twist_msg.linear.x = -min(self.linear_error, max_linear_vel) 
        self.cmd_vel_pub.publish(twist_msg)


    def dock_control_callback(self, request, response):
        self.desired_yaw = request.orientation
        self.is_angular_aligned = False
        self.is_docking = True
        self.docking_complete = False

        # Wait for docking completion
        rate = self.create_rate(100, self.get_clock())
        while not self.docking_complete:
            self.get_logger().info("Docking in progress...")
            rate.sleep()

        response.success = True
        response.message = "Docking completed"
        return response


def main(args=None):
    rclpy.init(args=args)
    my_robot_docking_controller = MyRobotDockingController()
    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)
    executor.spin()
    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
