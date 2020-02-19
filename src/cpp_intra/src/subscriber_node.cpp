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

#include <cinttypes>

#include "cpp_intra/common.h"
#include "cpp_intra/subscriber_node.hpp"

using Contact = tutorials_msgs::msg::Contact;

SubscriberNode::SubscriberNode(rclcpp::NodeOptions options)
        : Node("subscriber_node", options) {
    subscription_ = create_subscription<Contact>(
            "topic",
            10,
            [this](Contact::UniquePtr msg) {
                RCLCPP_INFO(this->get_logger(), "Thread %s received message with value: %s, and address: 0x%"
                PRIXPTR
                "\n", string_thread_id().c_str(), msg->address.c_str(),
                        reinterpret_cast<std::uintptr_t>(msg.get()));
            });
}
