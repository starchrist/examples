// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../include/minimal_publisher_autoware/minimal_publisher_autoware_node.hpp"

using namespace std::chrono_literals;

namespace autoware
{
namespace minimal_publisher_autoware
{

MinimalPublisherAutowareNode::MinimalPublisherAutowareNode(const rclcpp::NodeOptions & options)
:  Node("minimal_publisher_autoware", options),
  verbose(true),
  count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(
    500ms, std::bind(&MinimalPublisherAutowareNode::timer_callback, this));
}

void MinimalPublisherAutowareNode::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(MinimalPublisherAutowareNode::coeff(count_++));
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

size_t MinimalPublisherAutowareNode::coeff(size_t input)
{
  return minimal_publisher_autoware::coeff(input);
}

}  // namespace minimal_publisher_autoware
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimal_publisher_autoware::MinimalPublisherAutowareNode)
