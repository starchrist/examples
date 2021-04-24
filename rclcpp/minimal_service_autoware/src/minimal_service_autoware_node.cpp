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

#include <iostream>

#include "minimal_service_autoware/minimal_service_autoware_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace autoware
{
namespace minimal_service_autoware
{

MinimalServiceAutowareNode::MinimalServiceAutowareNode(const rclcpp::NodeOptions & options)
:  Node("minimal_service_autoware", options),
  verbose(true)
{
  server_ = this->create_service<AddTwoInts>("add_two_ints", 
  // Spend a lot of time here, just want to know the methodolgy to pas this point 
  // into call back. std::bind methodology which is the new feature of c++ 11 is
  // super useful here!! Code is clean and makes me happy.
  // remember the place holder is important here, in order to have the same input as callback.
  std::bind(&MinimalServiceAutowareNode::handle_service, this, _1, _2, _3));
}

void MinimalServiceAutowareNode::handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<AddTwoInts::Request> request,
  const std::shared_ptr<AddTwoInts::Response> response
)
{
  (void)request_header;
  RCLCPP_INFO(
    this->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b);
  response->sum = request->a + request->b;
}

}  // namespace minimal_service_autoware
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimal_service_autoware::MinimalServiceAutowareNode)