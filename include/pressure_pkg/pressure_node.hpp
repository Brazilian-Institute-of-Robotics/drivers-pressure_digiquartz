#pragma once

#include "pressure_pkg/pressure_driver.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

class PressureNode : public rclcpp::Node
{
  public:
    PressureNode();

  private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pressure_publisher_;
    pressure_pkg::PressureDriver pressure_driver_;
};
