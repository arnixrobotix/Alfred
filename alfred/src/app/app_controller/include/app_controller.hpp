// Copyright (c) 2024 Arnix Robotix
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

#ifndef APP_CONTROLLER_HPP_
#define APP_CONTROLLER_HPP_

#include "common.hpp"

// Services and messages headers (generated)
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/wrench.hpp"

namespace app
{
namespace controller
{

using OdometryMsg_t = nav_msgs::msg::Odometry;
using WrenchMsg_t = geometry_msgs::msg::Wrench;

class Controller : public rclcpp_lifecycle::LifecycleNode
{
private:
  rclcpp::Subscription<OdometryMsg_t>::SharedPtr odometrySubscriber;
  rclcpp_lifecycle::LifecyclePublisher<WrenchMsg_t>::SharedPtr commandPublisher;

  float previousCommand;
  float previousError;

public:
  Controller();
  ~Controller() = default;

  LifecycleCallbackReturn_t on_configure(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_activate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_deactivate(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_cleanup(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_shutdown(const rclcpp_lifecycle::State & previous_state);
  LifecycleCallbackReturn_t on_error(const rclcpp_lifecycle::State & previous_state);

  void odometryReader(const OdometryMsg_t & msg);
  void computeAndPublishCommand(double angle);
};

}  // namespace controller
}  // namespace app

#endif  // APP_CONTROLLER_HPP_
