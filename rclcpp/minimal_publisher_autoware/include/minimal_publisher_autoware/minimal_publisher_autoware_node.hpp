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
/// \brief This file defines the minimal_publisher_autoware_node class.

#ifndef MINIMAL_PUBLISHER_AUTOWARE__MINIMAL_PUBLISHER_AUTOWARE_NODE_HPP_
#define MINIMAL_PUBLISHER_AUTOWARE__MINIMAL_PUBLISHER_AUTOWARE_NODE_HPP_

#include <chrono>
#include <memory>

#include <minimal_publisher_autoware/minimal_publisher_autoware.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace autoware
{
namespace minimal_publisher_autoware
{

/// \class MinimalPublisherAutowareNode
/// \brief ROS 2 Node for minimal publisher.
class MINIMAL_PUBLISHER_AUTOWARE_PUBLIC MinimalPublisherAutowareNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit MinimalPublisherAutowareNode(const rclcpp::NodeOptions & options);

private:
  bool verbose;  ///< whether to use verbose output or not.
  void timer_callback(); // define time callback here
  size_t coeff(size_t input);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};
}  // namespace minimal_publisher_autoware
}  // namespace autoware

#endif  // MINIMAL_PUBLISHER_AUTOWARE__MINIMAL_PUBLISHER_AUTOWARE_NODE_HPP_
