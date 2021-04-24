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
/// \brief This file defines the minimal_service_autoware_node class.

#ifndef MINIMAL_SERVICE_AUTOWARE__MINIMAL_SERVICE_AUTOWARE_NODE_HPP_
#define MINIMAL_SERVICE_AUTOWARE__MINIMAL_SERVICE_AUTOWARE_NODE_HPP_
#include <inttypes.h>
#include <memory>
#include <minimal_service_autoware/minimal_service_autoware.hpp>

#include <rclcpp/rclcpp.hpp>

#include "example_autoware_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = example_autoware_interfaces::srv::AddTwoInts;

namespace autoware
{
namespace minimal_service_autoware
{

/// \class MinimalServiceAutowareNode
/// \brief ROS 2 Node for hello world.
class MINIMAL_SERVICE_AUTOWARE_PUBLIC MinimalServiceAutowareNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit MinimalServiceAutowareNode(const rclcpp::NodeOptions & options);

private:
  bool verbose;  ///< whether to use verbose output or not.
  rclcpp::Service<AddTwoInts>::SharedPtr server_;
  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response
);
};



}  // namespace minimal_service_autoware
}  // namespace autoware

#endif  // MINIMAL_SERVICE_AUTOWARE__MINIMAL_SERVICE_AUTOWARE_NODE_HPP_
