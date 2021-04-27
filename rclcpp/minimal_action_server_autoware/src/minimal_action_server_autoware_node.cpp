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
#include <memory>
#include "minimal_action_server_autoware/minimal_action_server_autoware_node.hpp"
using namespace std::placeholders;

namespace autoware
{
namespace minimal_action_server_autoware
{
MinimalActionServerAutowareNode::MinimalActionServerAutowareNode(const rclcpp::NodeOptions & options)
:  Node("minimal_action_server_autoware", options)
{
  this->m_action_server = rclcpp_action::create_server<Fibonacci>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "fibonacci",
      std::bind(&MinimalActionServerAutowareNode::handle_goal, this, _1, _2),
      std::bind(&MinimalActionServerAutowareNode::handle_cancel, this, _1),
      std::bind(&MinimalActionServerAutowareNode::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse MinimalActionServerAutowareNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->order > 9000) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

rclcpp_action::CancelResponse MinimalActionServerAutowareNode::handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

void MinimalActionServerAutowareNode::handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // this is an interesting part, if client is requiring a very fast response, it is better to issue execute in
    // a new thread, otherwise client may not be able to have response. 
    // However, if client is requiring crazy frequency, thread number may reach the maximum value 4096 and cause crash.
    std::thread{std::bind(&MinimalActionServerAutowareNode::execute, this, _1), goal_handle}.detach();
  }

void MinimalActionServerAutowareNode::execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    // https://blog.csdn.net/qq825255961/article/details/105471390
    // Rate's usage can be checked in above website
    // Attetion: numer's definition here is rate, unit is in Hz.
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    // check Fibonacci.action Feedback.sequence is a int32[] vector.
    auto & sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result = std::make_shared<Fibonacci::Result>();

    // it is necessary to set correct index type, otherwise autoware compiling configuration
    // will check the calculation between signed/unsigned variable
    std::vector<int>::size_type index = 1;
    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        // check Fibonacci.action Feedback.sequence is a int32[] vector.
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[index] + sequence[index - 1]);
      index++;
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }


}  // namespace minimal_action_server_autoware
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimal_action_server_autoware::MinimalActionServerAutowareNode)
