#!/usr/bin/env python3
#coding:utf-8

# Ref : https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TopicPublisher(Node):
    def __init__(self):
        super().__init__('topic_publisher_template')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = String()
        self.declare_parameter( 'message', 'Hello, World!')
        self.msg.data = self.get_parameter("message").get_parameter_value().string_value

    def timer_callback(self):
        self.publisher_.publish(self.msg)
        self.get_logger().info('Message Publishing: "%s"' % self.msg.data)
        return

def main(args=None):
    rclpy.init(args=args)
    topic_publisher = TopicPublisher()
    try:
        rclpy.spin(topic_publisher)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    topic_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()