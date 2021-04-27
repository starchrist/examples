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
/// \brief This file defines the minimal_action_server_autoware_node class.

#ifndef MINIMAL_ACTION_SERVER_AUTOWARE__MINIMAL_ACTION_SERVER_AUTOWARE_NODE_HPP_
#define MINIMAL_ACTION_SERVER_AUTOWARE__MINIMAL_ACTION_SERVER_AUTOWARE_NODE_HPP_

#include "example_autoware_interfaces/action/fibonacci.hpp"
#include <minimal_action_server_autoware/minimal_action_server_autoware.hpp>

#include <rclcpp/rclcpp.hpp>
// TODO : Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"


namespace autoware
{
namespace minimal_action_server_autoware
{

/// \class MinimalActionServerAutowareNode
/// \brief ROS 2 Node for hello world.
class MINIMAL_ACTION_SERVER_AUTOWARE_PUBLIC MinimalActionServerAutowareNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  using Fibonacci = example_autoware_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
  explicit MinimalActionServerAutowareNode(const rclcpp::NodeOptions & options);
  
private:
  rclcpp_action::Server<Fibonacci>::SharedPtr m_action_server;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle);
  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle);
  
};
}  // namespace minimal_action_server_autoware
}  // namespace autoware

#endif  // MINIMAL_ACTION_SERVER_AUTOWARE__MINIMAL_ACTION_SERVER_AUTOWARE_NODE_HPP_
