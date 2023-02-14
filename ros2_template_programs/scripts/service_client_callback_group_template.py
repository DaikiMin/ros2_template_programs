#!/usr/bin/env python3
#coding:utf-8

from ros2_custom_msg.msg import Expression
from ros2_custom_msg.srv import Calculation
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class ServiceClient(Node):

    def __init__(self):
        super().__init__('service_client_callback_group_template')

        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.client = self.create_client(Calculation, 'calculate_two_numbers', callback_group=self.client_cb_group)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=self.timer_cb_group)
        self.declare_parameter( 'a', 1.0)
        self.declare_parameter( 'b', 1.0)
        self.count = 0
        self.req = Calculation.Request()
        self.req.expression.a = self.get_parameter("a").value
        self.req.expression.b = self.get_parameter("b").value
        self.client_futures = []

    async def send_request(self):
        if self.count%4 == 0:
            self.req.expression.calculate = '+'
        elif self.count%4 == 1:
            self.req.expression.calculate = '-'
        elif self.count%4 == 2:
            self.req.expression.calculate = '*'
        else:
            self.req.expression.calculate = '/'
        return await self.client.call_async(self.req)

    async def timer_callback(self) -> None:
        response = await self.send_request()
        try:
            self.get_logger().info('Result: %.2f\n' % response.result)
            self.count += 1
        except Exception as e:
            self.get_logger().info(f"Service call failed {e}")
        return

def main():
    rclpy.init()
    client = ServiceClient()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(client)
    try:
        client.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        client.get_logger().info('Keyboard interrupt, shutting down.\n')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()