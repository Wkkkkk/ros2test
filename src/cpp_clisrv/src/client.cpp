//
// Created by zhihui on 1/14/20.
//

#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorials_msgs/srv/add_two_floats.hpp"

using namespace std::chrono_literals;
using AddTwoFloats = tutorials_msgs::srv::AddTwoFloats;


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc != 3) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        return 1;
    }

    std::shared_ptr <rclcpp::Node> node = rclcpp::Node::make_shared("minimal_client");
    rclcpp::Client<AddTwoFloats>::SharedPtr client = node->create_client<AddTwoFloats>("add_two_floats");

    auto request = std::make_shared<AddTwoFloats::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service available, request is being sent...");
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    rclcpp::shutdown();
    return 0;
}