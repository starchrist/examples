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
/// \brief This file defines the minimal_client_autoware_node class.

#ifndef MINIMAL_CLIENT_AUTOWARE__MINIMAL_CLIENT_AUTOWARE_NODE_HPP_
#define MINIMAL_CLIENT_AUTOWARE__MINIMAL_CLIENT_AUTOWARE_NODE_HPP_

#include <minimal_client_autoware/minimal_client_autoware.hpp>

#include <rclcpp/rclcpp.hpp>
#include "example_autoware_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_autoware_interfaces::srv::AddTwoInts;
using ServiceResponseFuture =
	    rclcpp::Client<example_autoware_interfaces::srv::AddTwoInts>::SharedFuture;

namespace autoware
{
namespace minimal_client_autoware
{

/// \class MinimalClientAutowareNode
/// \brief ROS 2 Node for hello world.
class MINIMAL_CLIENT_AUTOWARE_PUBLIC MinimalClientAutowareNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit MinimalClientAutowareNode(const rclcpp::NodeOptions & options);

private:
  bool verbose;  ///< whether to use verbose output or not.
  int32_t m_cout;
  rclcpp::Client<AddTwoInts>::SharedPtr m_client;
  rclcpp::TimerBase::SharedPtr m_timer;
  void minimal_client_callback();
  void response_callback(ServiceResponseFuture result_future);
};
}  // namespace minimal_client_autoware
}  // namespace autoware

#endif  // MINIMAL_CLIENT_AUTOWARE__MINIMAL_CLIENT_AUTOWARE_NODE_HPP_
