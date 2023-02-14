#ifndef SERVICE_CLIENT_FROM_CALLBACK_TEMPLATE_HPP
#define SERVICE_CLIENT_FROM_CALLBACK_TEMPLATE_HPP

#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_custom_msg/msg/expression.hpp"
#include "ros2_custom_msg/srv/calculation.hpp"

using namespace std::chrono_literals;

namespace ros2_template_programs {
    class ServiceClient  : public rclcpp::Node {
        private:
            rclcpp::CallbackGroup::SharedPtr client_cb_group_;
            rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Client<ros2_custom_msg::srv::Calculation>::SharedPtr client_;
            ros2_custom_msg::srv::Calculation::Request::SharedPtr request_;
            int count_;
            void callbackTimer();

        public:
            ServiceClient ();
            void sendRequest();
    };
}

#endif