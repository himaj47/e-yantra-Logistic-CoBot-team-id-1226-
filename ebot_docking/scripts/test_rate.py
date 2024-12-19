import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import threading

class testNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.counter = 0

        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.control_loop)
        # self.timer2 = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback2, callback_group=ReentrantCallbackGroup())

        self.is_docking = False
        self.dock_aligned = False

        thread = threading.Thread(target=self.timer_callback2)
        thread.start()

        # while True:
        # self.timer_callback2()

    def control_loop(self):
        if self.is_docking:
            self.counter += 1
            if self.counter >= 3:
                self.dock_aligned = True

            self.get_logger().info("counter: " + str(self.counter))

    def timer_callback2(self):

        self.is_docking = True

        rate = self.create_rate(2, self.get_clock())
        
        while not self.dock_aligned:
            self.get_logger().info("Waiting for alignment...")
            rate.sleep() 

        # while self.dock_aligned:
        #     self.get_logger().info("Waiting for alignment...")
        #     # rate.sleep() 

        print("return")
        # rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    # executor = rclpy.executors.MultiThreadedExecutor()
    node = testNode()
    # executor.add_node(node)
    # executor.spin()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()