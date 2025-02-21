#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class Resetloca(Node):
    def __init__(self):

        super().__init__('reset')
        self.imu=self.create_client(Trigger,'/reset_imu')
        self.odom=self.create_client(Trigger,'/reset_odom')
        

        while not self.imu.wait_for_service(1.0):
            self.get_logger().info(f'waiting for imu seervice ')
        
        while not self.odom.wait_for_service(1.0):
            self.get_logger().info(f'waiting for odom seervice ')
        self.reset_call=False
        self.call_service()

    def call_service(self):
        print('In the Call Service ')

        if not self.reset_call:
            print('Trying to Call')
            # if self.imu.wait_for_service(1.0):
            future =self.imu.call_async(Trigger.Request())
            print(future.result())

            # if self.odom.wait_for_service(1.0):
            res=self.odom.call_async(Trigger.Request())
            print(res.result())
            self.reset_call=True
            # print('')



def main(args=None):
    rclpy.init(args=args)
    node=Resetloca()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()