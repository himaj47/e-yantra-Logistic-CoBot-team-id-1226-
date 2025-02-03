import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

class NetWrench(Node):
    def __init__(self):
        super().__init__("check_net_wrench")

        self.create_subscription(Float64, "/net_wrench", self.callBack, 10)

    def callBack(self, msg:Float64):
        print(f"net_wrench = {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = NetWrench()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()