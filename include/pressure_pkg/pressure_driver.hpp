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

#include <string>
#include <iostream>
#include <boost/asio.hpp>

#include "ros_driver_base/driver.hpp"
#include "pressure_pkg/visibility_control.h"

namespace pressure_pkg
{
class PressureDriver : public ros_driver_base::Driver
{
public:
  PressureDriver();
  virtual ~PressureDriver();

  double getPressure();
  bool read(std::string & response);

  static const int PRESSURE_MAX_PACKET_SIZE{32};
  uint8_t PRESSURE_PACKET[32];

private:
  boost::asio::io_service io_service_;
  std::string pattern_;
  int extractPacket(const uint8_t * buffer, size_t buffer_size) const override;
  mutable double pressure_value_ = 0.0;
};
}  // namespace pressure_pkg
