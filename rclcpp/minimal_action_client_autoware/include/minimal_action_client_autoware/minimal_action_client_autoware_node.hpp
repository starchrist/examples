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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the minimal_action_client_autoware_node class.

#ifndef MINIMAL_ACTION_CLIENT_AUTOWARE__MINIMAL_ACTION_CLIENT_AUTOWARE_NODE_HPP_
#define MINIMAL_ACTION_CLIENT_AUTOWARE__MINIMAL_ACTION_CLIENT_AUTOWARE_NODE_HPP_

#include "example_autoware_interfaces/action/fibonacci.hpp"

#include <minimal_action_client_autoware/minimal_action_client_autoware.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace autoware
{
namespace minimal_action_client_autoware
{

/// \class MinimalActionClientAutowareNode
/// \brief ROS 2 Node for hello world.
class MINIMAL_ACTION_CLIENT_AUTOWARE_PUBLIC MinimalActionClientAutowareNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  using Fibonacci = example_autoware_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
  explicit MinimalActionClientAutowareNode(const rclcpp::NodeOptions & options);
  
private:
  bool m_goal_done;  ///< whether to use verbose output or not.
  rclcpp_action::Client<Fibonacci>::SharedPtr m_client_ptr;
  rclcpp::TimerBase::SharedPtr m_timer;
  void send_goal();
  void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future);
  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback);
  void result_callback(const GoalHandleFibonacci::WrappedResult & result);
};
}  // namespace minimal_action_client_autoware
}  // namespace autoware

#endif  // MINIMAL_ACTION_CLIENT_AUTOWARE__MINIMAL_ACTION_CLIENT_AUTOWARE_NODE_HPP_
