#ifndef TOPIC_PUBLISHER_TEMPLATE_HPP
#define TOPIC_PUBLISHER_TEMPLATE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace ros2_template_programs {
    class TopicPublisher  : public rclcpp::Node {
        private:
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_msg_ ;
            std_msgs::msg::String msg_;
            void callbackTimer();

        public:
            TopicPublisher ();
    };
}

#endif