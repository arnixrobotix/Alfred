// Copyright (c) 2022 Arnix Robotix
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "gpioDefinitions.hpp"

struct Quaternion
{
  double w;
  double x;
  double y;
  double z;
};

struct Vector3
{
  double x;
  double y;
  double z;
};

struct Twist
{
  Vector3 linear;
  Vector3 angular;
};

struct Pose
{
  Vector3 position;
  Quaternion orientation;
};

struct Odometry
{
  Twist twist;
  Pose pose;
};

enum class EdgeChangeConfiguration
{
  asRisingEdge,
  asFallingEdge,
  asEitherEdge
};

enum class EncoderChannel
{
  channelA,
  channelB,
  undefined
};

enum class MotorDirection
{
  backward,
  forward,
  undefined
};

enum class EdgeChangeType
{
  falling,
  rising,
  no_change,
  undefined
};

template<class T>
inline bool AreEqual(T a, T b, T epsilon)
{
  return std::fabs(a - b) < epsilon;
}

using LifecycleCallbackReturn_t =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

template<class ServiceT>
class ServiceNodeSync
{
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;

public:
  explicit ServiceNodeSync(std::string name)
  : node(std::make_shared<rclcpp::Node>(name, rclcpp::NodeOptions().use_global_arguments(false))) {}

  ~ServiceNodeSync()
  {
    node.reset();
    client.reset();
  }

  void init(std::string service)
  {
    client = node->create_client<ServiceT>(service);
    client->wait_for_service();
  }

  ResponseT sendRequest(const RequestT & request)
  {
    return sendRequest(std::make_shared<RequestT>(request));
  }

  std::shared_ptr<ResponseT> sendRequest(const std::shared_ptr<RequestT> & request_ptr)
  {
    std::shared_ptr<ResponseT> response;
    auto result = client->async_send_request(request_ptr);
    auto status = rclcpp::spin_until_future_complete(
      node, result, std::chrono::duration<int64_t, std::milli>(1000));

    if (status == rclcpp::FutureReturnCode::SUCCESS) {
      response = result.get();
    }

    return response;
  }

protected:
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
};


#endif  // COMMON_HPP_
