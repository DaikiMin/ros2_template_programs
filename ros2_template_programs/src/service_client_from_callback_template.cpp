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

void ros2_template_programs::ServiceClient::callbackTimer() {
    if ( count_%4 == 0 ) {
        request_->expression.calculate = "+";
    } else if ( count_%4 == 1 ) {
        request_->expression.calculate = "-";
    } else if ( count_%4 == 2 ) {
        request_->expression.calculate = "*";
    } else {
        request_->expression.calculate = "/";
    }

    auto response_received_callback = [this](rclcpp::Client<ros2_custom_msg::srv::Calculation>::SharedFuture future) {
        // Wait for the result.
        auto status = future.wait_for(1ms);
        if (status == std::future_status::ready) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Call the calculate_two_numbers server\nExpression :%.2f %s %.2f\nResult : %.2f",
        request_->expression.a, request_->expression.calculate.c_str(), request_->expression.b, future.get()->result );
        } else {
            RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
        }
    };
    auto result = client_->async_send_request(request_, response_received_callback);
    count_++;
}


ros2_template_programs::ServiceClient::ServiceClient() : Node("service_client_from_callback_template") {
    client_ = this->create_client<ros2_custom_msg::srv::Calculation>("calculate_two_numbers");
    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        else RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    timer_ = this->create_wall_timer(1s, std::bind(&ServiceClient::callbackTimer, this));
    declare_parameter( "a", 1.0);
    declare_parameter( "b", 1.0);
    request_ = std::make_shared<ros2_custom_msg::srv::Calculation::Request>();
    request_->expression.a = get_parameter("a").as_double();
    request_->expression.b = get_parameter("b").as_double();
    count_ = 0;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ros2_template_programs::ServiceClient>());
    rclcpp::shutdown();
    return 0;
}