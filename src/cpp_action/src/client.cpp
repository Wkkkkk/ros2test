//
// Created by zhihui on 1/14/20.
//

#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include "tutorials_msgs/action/fibonacci.hpp"

class MinimalActionClient : public rclcpp::Node {
public:
    using Fibonacci = tutorials_msgs::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

    explicit MinimalActionClient(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
            : Node("minimal_action_client", node_options), goal_done_(false) {
        this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
                this->get_node_base_interface(),
                this->get_node_graph_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                "fibonacci");

        this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&MinimalActionClient::send_goal, this));

        this->timer2_ = this->create_wall_timer(
                std::chrono::milliseconds(5000),
                std::bind(&MinimalActionClient::cancel_goal, this));
    }

    bool is_goal_done() const {
        return this->goal_done_;
    }

    void cancel_goal() {
        this->timer_->cancel();

        auto cancel_result_future = this->client_ptr_->async_cancel_goal(goal_handle_future_.get());
    }

    void send_goal() {
        using namespace std::placeholders;

        this->timer_->cancel();

        this->goal_done_ = false;

        if (!this->client_ptr_) {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.order = 10;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
                std::bind(&MinimalActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
                std::bind(&MinimalActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
                std::bind(&MinimalActionClient::result_callback, this, _1);
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer2_;
    std::shared_future <std::shared_ptr<GoalHandleFibonacci>> goal_handle_future_;

    bool goal_done_;

    void goal_response_callback(std::shared_future <GoalHandleFibonacci::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            goal_handle_future_ = future;
        }
    }

    void feedback_callback(
            GoalHandleFibonacci::SharedPtr,
            const std::shared_ptr<const Fibonacci::Feedback> feedback) {
        RCLCPP_INFO(
                this->get_logger(),
                "Next number in sequence received: %"
        PRId64,
                feedback->partial_sequence.back());
    }

    void result_callback(const GoalHandleFibonacci::WrappedResult &result) {
        this->goal_done_ = true;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }

        RCLCPP_INFO(this->get_logger(), "Result received");
        for (auto number : result.result->sequence) {
            RCLCPP_INFO(this->get_logger(), "%"
            PRId64, number);
        }
    }
};  // class MinimalActionClient

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<MinimalActionClient>();

    while (!action_client->is_goal_done()) {
        rclcpp::spin_some(action_client);
    }

    rclcpp::shutdown();
    return 0;
}
