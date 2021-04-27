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
#include <inttypes.h>
#include "minimal_action_client_autoware/minimal_action_client_autoware_node.hpp"
using namespace std::placeholders;
namespace autoware
{
namespace minimal_action_client_autoware
{

MinimalActionClientAutowareNode::MinimalActionClientAutowareNode(const rclcpp::NodeOptions & options)
:  Node("minimal_action_client_autoware", options)
{
  this->m_client_ptr = rclcpp_action::create_client<Fibonacci>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fibonacci");
  // Create a timer to delay(with timer cancel)/periodically generate send_goal request to server.
  this->m_timer = this->create_wall_timer(
      std::chrono::milliseconds(5000),
      std::bind(&MinimalActionClientAutowareNode::send_goal, this));
}

void MinimalActionClientAutowareNode::send_goal()
{
  // cancel timer when entering callback
  // in demo codes, create_wall_timer is used as delay
  // when coder set [duration], and cancel it in callback, 
  // create_wall_timer will delay [duration] when construction function entered
  // and cancelled later. If periodic sending required, this timer cancel shall be removed.
  this->m_timer->cancel();
  // initialize goal status
  this->m_goal_done = false;
  
  // issue exceptions
  if (!this->m_client_ptr) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!this->m_client_ptr->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    this->m_goal_done = true;
    return;
  }

  auto goal_msg = Fibonacci::Goal();
  goal_msg.order = 10;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&MinimalActionClientAutowareNode::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&MinimalActionClientAutowareNode::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&MinimalActionClientAutowareNode::result_callback, this, _1);
  // send goal to server with different callbacks
  auto goal_handle_future = this->m_client_ptr->async_send_goal(goal_msg, send_goal_options);
}

void MinimalActionClientAutowareNode::goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void MinimalActionClientAutowareNode::feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Next number in sequence received: %" PRId64,
      feedback->sequence.back());
  }

  void MinimalActionClientAutowareNode::result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    // when result_callback is issued, goal_done status can be set to true.
    this->m_goal_done = true;
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
      RCLCPP_INFO(this->get_logger(), "%" PRId64, number);
    }
  }

}  // namespace minimal_action_client_autoware
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimal_action_client_autoware::MinimalActionClientAutowareNode)
