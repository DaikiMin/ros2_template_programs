#!/usr/bin/env python3
#coding:utf-8

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from ros2_custom_msg.action import Timer

class TimerActionServer(Node):
    def __init__(self):
        super().__init__('action_server_template')
        self._action_server = ActionServer( self, Timer, 'timer_action_server', self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Start Timer...')

        feedback = Timer.Feedback()
        result = Timer.Result()
        init_time = time.time()
        curt_time = init_time
        target_time = goal_handle.request.target_time

        while ( (curt_time - init_time) < target_time ) :
            curt_time = time.time()
            feedback.elapsed_time = curt_time - init_time
            goal_handle.publish_feedback( feedback )
            self.get_logger().info('Publish the feedback : elapsed_time = %.2f' % feedback.elapsed_time )
            time.sleep(0.1)

        result.total_time = feedback.elapsed_time
        self.get_logger().info('It is time to end : total_time = %.2f\n' % result.total_time )
        goal_handle.succeed()
        return result

def main(args=None):
    rclpy.init()
    action_server = TimerActionServer()
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()