#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_custom_msg/msg/Expression.hpp"
#include "ros2_custom_msg/srv/Calculation.hpp"


using std::placeholders::_1;

namespace ros2_template_programs {
    class ServiceServer : public rclcpp::Node {
        private:
            rclcpp::Service<ros2_custom_msg::srv::Calculation>::SharedPtr service_;
            void calculate(
                const std::shared_ptr<ros2_custom_msg::srv::Calculation::Request> request,
                std::shared_ptr<ros2_custom_msg::srv::Calculation::Response>      response) const;
        public:
            ServiceServer();
    };
}

void ros2_template_programs::ServiceServer::calculate(
    const std::shared_ptr<ros2_custom_msg::srv::Calculation::Request> request,
    std::shared_ptr<ros2_custom_msg::srv::Calculation::Response>      response) const {
    ros2_custom_msg::msg::Expression ex = request->expression;
    if ( ex.calculate == "+" ) {
        response->result = ex.a + ex.b;
    } else if ( ex.calculate == "-" ) {
        response->result = ex.a - ex.b;
    } else if ( ex.calculate == "*" ) {
        response->result = ex.a * ex.b;
    } else {
        response->result = ex.a / ex.b;
    }
    RCLCPP_INFO(this->get_logger(), "Received a call from a client\nRequest: %.2f %s %.2f", ex.a, ex.calculate.c_str(), ex.b);
    RCLCPP_INFO(this->get_logger(), "sending back response: [%.f]\n", response->result);
}

ros2_template_programs::ServiceServer::ServiceServer() : Node("service_server_template") {
    service_ = this->create_service<ros2_custom_msg::srv::Calculation>("calculate_two_numbers", std::bind(&ServiceServer::calculate, this, _1, _2));
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros2_template_programs::ServiceServer>());
    rclcpp::shutdown();
    return 0;
}