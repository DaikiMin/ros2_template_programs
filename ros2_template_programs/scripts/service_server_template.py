#!/usr/bin/env python3
#coding:utf-8

from ros2_custom_msg.msg import Expression
from ros2_custom_msg.srv import Calculation
import rclpy
from rclpy.node import Node

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server_template')
        self.srv = self.create_service(Calculation, 'calculate_two_numbers', self.calculate)

    def calculate(self, request, response):
        ex = request.expression
        if ex.calculate == '+':
            response.result = ex.a + ex.b
        elif ex.calculate == '-':
            response.result = ex.a - ex.b
        elif ex.calculate == '/':
            response.result = ex.a * ex.b
        else:
            response.result = ex.a / ex.b
        self.get_logger().info('Received a call from a client\nRequest: %.2f %s %.2f = %.2f\n' % (ex.a, ex.calculate, ex.b, response.result))
        return response

def main():
    rclpy.init()
    server = ServiceServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()