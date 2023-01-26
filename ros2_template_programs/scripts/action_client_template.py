#!/usr/bin/env python3
#coding:utf-8

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ros2_custom_msg.action import Timer


class TimerActionClient(Node):
    def __init__(self):
        super().__init__('action_client_template')
        self._action_client = ActionClient(self, Timer, 'timer_action_server')
        self.action_executing = False

    def send_goal(self, target_time):
        goal_msg = Timer.Goal()
        goal_msg.target_time = target_time
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Received result : Total_time = %.2f\n' % result.total_time )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback : elapsed_time = %.2f' % feedback.elapsed_time)

def main(args=None):
    rclpy.init(args=args)
    action_client = TimerActionClient()
    action_client.declare_parameter( 'target_time', 1.0)
    try:
        action_client.send_goal(action_client.get_parameter('target_time').value)
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()