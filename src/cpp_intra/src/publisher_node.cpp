// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>

#include "cpp_intra/common.h"
#include "cpp_intra/publisher_node.hpp"

using namespace std::chrono_literals;
using Contact = tutorials_msgs::msg::Contact;

PublisherNode::PublisherNode(rclcpp::NodeOptions options)
        : Node("publisher_node", options), count_(0) {
    publisher_ = create_publisher<Contact>("topic", 10);
    timer_ = create_wall_timer(
            500ms, std::bind(&PublisherNode::on_timer, this));
}

void PublisherNode::on_timer() {
    auto message = std::make_unique<Contact>();
    message->address = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Thread %s published message with value: %s, and address: 0x%"
    PRIXPTR
    "\n", string_thread_id().c_str(), message->address.c_str(),
            reinterpret_cast<std::uintptr_t>(message.get()));
    publisher_->publish(std::move(message));
}
