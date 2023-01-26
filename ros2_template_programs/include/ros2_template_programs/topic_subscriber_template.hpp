#ifndef TOPIC_SUBSCRIBER_TEMPLATE_HPP
#define TOPIC_SUBSCRIBER_TEMPLATE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros2_template_programs {
    class TopicSubscriber : public rclcpp::Node {
        private:
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_msg_;
            void callbackMessage(const std_msgs::msg::String & msg) const;
        public:
            TopicSubscriber();
    };
}

#endif