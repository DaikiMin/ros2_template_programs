#ifndef SERVICE_SERVER_TEMPLATE_HPP
#define SERVICE_SERVER_TEMPLATE_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_custom_msg/msg/expression.hpp"
#include "ros2_custom_msg/srv/calculation.hpp"

namespace ros2_template_programs {
    class ServiceServer : public rclcpp::Node {
        private:
            rclcpp::Service<ros2_custom_msg::srv::Calculation>::SharedPtr service_;
            void calculate( const std::shared_ptr<ros2_custom_msg::srv::Calculation::Request> request, std::shared_ptr<ros2_custom_msg::srv::Calculation::Response> response) const;
        public:
            ServiceServer();
    };
}

#endif