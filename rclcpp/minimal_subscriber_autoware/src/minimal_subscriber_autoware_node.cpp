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

#include "minimal_subscriber_autoware/minimal_subscriber_autoware_node.hpp"

using std::placeholders::_1;

namespace autoware
{
namespace minimal_subscriber_autoware
{

MinimalSubscriberAutowareNode::MinimalSubscriberAutowareNode(const rclcpp::NodeOptions & options)
:  Node("minimal_subscriber_autoware", options),
  verbose(true)
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriberAutowareNode::topic_callback, this, _1));
}

void MinimalSubscriberAutowareNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

}  // namespace minimal_subscriber_autoware
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimal_subscriber_autoware::MinimalSubscriberAutowareNode)
