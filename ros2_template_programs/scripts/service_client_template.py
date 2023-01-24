#!/usr/bin/env python3
#coding:utf-8

from ros2_custom_msg.msg import Expression
from ros2_custom_msg.srv import Calculation
import rclpy
from rclpy.node import Node

class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client_template')
        self.client = self.create_client(Calculation, 'calculate_two_numbers')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.declare_parameter( 'a', 1.0)
        self.declare_parameter( 'b', 1.0)
        self.count = 0
        self.req = Calculation.Request()
        self.req.expression.a = self.get_parameter("a").value
        self.req.expression.b = self.get_parameter("b").value

    def send_request(self):
        loop_rate = self.create_rate(1)
        while rclpy.ok():
            if self.count%4 == 0:
                self.req.expression.calculate = '+'
            elif self.count%4 == 1:
                self.req.expression.calculate = '-'
            elif self.count%4 == 2:
                self.req.expression.calculate = '*'
            else:
                self.req.expression.calculate = '/'
            self.future = self.client.call_async(self.req)
            rclpy.spin_until_future_complete(self, self.future)
            # See if the service has replied
            if self.future.done():
                try:
                    res = self.future.result()
                except Exception as e:
                    self.get_logger().info(f"Service call failed {e}")
                else:
                    self.get_logger().info('Result: %.2f\n' % res.result)
                    self.count += 1
            else:
                self.get_logger().info("Service call failed")
            # loop_rate.sleep()


def main():
    rclpy.init()
    client = ServiceClient()
    try:
        client.send_request()
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()