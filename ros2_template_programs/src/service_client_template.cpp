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
            rclcpp::Client<ros2_custom_msg::srv::Calculation>::SharedPtr client_;
            ros2_custom_msg::srv::Calculation::Request::SharedPtr request_;
            int count_;

        public:
            ServiceClient ();
            void sendRequest();
    };
}


ros2_template_programs::ServiceClient::ServiceClient() : Node("service_client_template") {
    client_ = this->create_client<ros2_custom_msg::srv::Calculation>("calculate_two_numbers");
    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) RCLCPP_ERROR(rclcpp::get_logger(), "Interrupted while waiting for the service. Exiting.");
        else RCLCPP_INFO(rclcpp::get_logger(), "service not available, waiting again...");
    }
    declare_parameter( "a", 1.0);
    declare_parameter( "b", 1.0);
    request_ = std::make_shared<ros2_custom_msg::srv::Calculation::Request>();
    request_->expression.a = get_parameter("a").as_double();
    request_->expression.b = get_parameter("b").as_double();
    count_ = 0;
}

void ros2_template_programs::ServiceClient::sendRequest() {
    while (rclcpp::ok()) {
        if ( count_%4 == 0 ) {
            request_->expression.calculate = "+";
        } else if ( count_%4 == 1 ) {
            request_->expression.calculate = "-";
        } else if ( count_%4 == 2 ) {
            request_->expression.calculate = "*";
        } else {
            request_->expression.calculate = "/";
        }

        auto result = client_->async_send_request(request_);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger(), "Call the calculate_two_numbers server\nExpression :%.2f %s %.2f\nResult : %.2f",
            request_->expression.a, request_->expression.calculate.c_str(), request_->expression.b, result.get()->result );
        } else {
            RCLCPP_ERROR(rclcpp::get_logger(), "Failed to call service calculate_two_numbers");
        }
        count_++;
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ros2_template_programs::ServiceClient>();
    client->sendRequest();
    rclcpp::shutdown();
    return 0;
}