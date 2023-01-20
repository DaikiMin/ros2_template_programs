#!/usr/bin/env python3
#coding:utf-8

# Ref : https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TopicSubscriber(Node):

    def __init__(self):
        super().__init__('topic_subscriber_template')
        self.subscription = self.create_subscription( String, 'topic', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Message Subscribed: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    topic_subscriber = TopicSubscriber()
    try:
        rclpy.spin(topic_subscriber)
    except KeyboardInterrupt:
            pass
    # Destroy the node explicitly
    topic_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()