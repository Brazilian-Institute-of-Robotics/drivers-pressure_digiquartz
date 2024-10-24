// Copyright (c) 2023, SENAI Cimatec
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

#pragma once

#include "pressure_pkg/pressure_driver.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class PressureNode
 * @brief Class responsible for publishing pressure data in a ROS 2 environment.
 * The PressureNode class creates a ROS 2 node that interacts with the PressureDriver to fetch pressure data
 * and publishes it on a topic as a `std_msgs::msg::Float64` message.
 */
class PressureNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new PressureNode object.
   * Initializes the ROS 2 node, sets up the pressure driver, and starts a timer for periodically publishing pressure data.
   */
  PressureNode();

private:
  /**
   * @brief Timer callback function for publishing pressure data.
   * This method is called periodically by the ROS 2 timer to fetch the current pressure value using the PressureDriver
   * and publish it on the designated topic.
   */
  void timer_callback();

  /**< Shared pointer to the ROS 2 timer object.*/
  rclcpp::TimerBase::SharedPtr timer_;

  /**< Publisher for the pressure data.*/
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pressure_publisher_;

  /**< PressureDriver instance for reading pressure data from the sensor.*/
  pressure_pkg::PressureDriver pressure_driver_;
};
