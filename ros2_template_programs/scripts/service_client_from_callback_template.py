#!/usr/bin/env python3
#coding:utf-8

from ros2_custom_msg.msg import Expression
from ros2_custom_msg.srv import Calculation
import rclpy
from rclpy.node import Node

class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client_from_callback_template')
        self.client = self.create_client(Calculation, 'calculate_two_numbers')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.declare_parameter( 'a', 1.0)
        self.declare_parameter( 'b', 1.0)
        self.count = 0
        self.req = Calculation.Request()
        self.req.expression.a = self.get_parameter("a").value
        self.req.expression.b = self.get_parameter("b").value
        self.client_futures = []

    def timer_callback(self):
        if self.count%4 == 0:
            self.req.expression.calculate = '+'
        elif self.count%4 == 1:
            self.req.expression.calculate = '-'
        elif self.count%4 == 2:
            self.req.expression.calculate = '*'
        else:
            self.req.expression.calculate = '/'
        self.client_futures.append(self.client.call_async(self.req))
        self.count += 1
        return

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:
                if f.done():
                    try:
                        res = f.result()
                    except Exception as e:
                        self.get_logger().info(f"Service call failed {e}")
                    else:
                        self.get_logger().info('Result: %.2f\n' % res.result)
                else:
                    incomplete_futures.append(f)
            self.client_futures = incomplete_futures

def main():
    rclpy.init()
    client = ServiceClient()
    try:
        client.spin()
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()