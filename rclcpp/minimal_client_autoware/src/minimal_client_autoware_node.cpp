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
#include <chrono>
#include <cinttypes>
#include <memory>
#include "minimal_client_autoware/minimal_client_autoware_node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace autoware
{
namespace minimal_client_autoware
{

MinimalClientAutowareNode::MinimalClientAutowareNode(const rclcpp::NodeOptions & options)
:  Node("minimal_client_autoware", options),
  verbose(true),
  m_cout(2)
{
  m_client = this->create_client<AddTwoInts>("add_two_ints");
  while (!m_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
  }
  m_timer = this->create_wall_timer(
    2000ms, std::bind(&MinimalClientAutowareNode::minimal_client_callback, this));
}

void MinimalClientAutowareNode::minimal_client_callback()
{
  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 41; request->b = 1 + m_cout++;

  auto result = m_client->async_send_request(
    request, 
    std::bind(&MinimalClientAutowareNode::response_callback, this, _1));
  
  // https://blog.csdn.net/liuerin/article/details/108104554
  // based on the blog above, it seems that <spin_until_future_complete> will
  // add node into a new executor, which is going to raise error in ROS2 running.
  // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
  //   rclcpp::executor::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_ERROR(this->get_logger(), "service call failed :(");
  // }
  
  // https://blog.csdn.net/liuerin/article/details/108104554
  // lambda function from blog, after test, with this call back, still have the problem from rclcpp:
  // [Received invalid sequence number.] Reason is not clear
  // with new build, this issue suddenly disappear.
  // auto response_received_callback = [this](ServiceResponseFuture future) {
  //     RCLCPP_INFO(this->get_logger(), "Got result: [%d]", future.get()->sum);
  //   };  
}

void MinimalClientAutowareNode::response_callback(ServiceResponseFuture result_future)
{
  auto msg = result_future.get();
  RCLCPP_INFO(this->get_logger(), "result = %" PRId64, msg->sum);
}

}  // namespace minimal_client_autoware
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimal_client_autoware::MinimalClientAutowareNode)
