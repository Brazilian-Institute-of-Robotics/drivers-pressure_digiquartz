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
/**
 * @class PressureDriver
 * @brief Class responsible for managing the communication with a pressure sensor and retrieving pressure data.
 * This class inherits from the ros_driver_base::Driver base class and provides functionality to read pressure values
 * from a sensor via serial communication. It utilizes Boost.Asio for handling asynchronous I/O operations.
 */
class PressureDriver : public ros_driver_base::Driver
{
public:
  /**
   * @brief Construct a new PressureDriver object.
   * Initializes the PressureDriver instance and sets up necessary configurations for communication.
   */
  PressureDriver();

  /**
   * @brief Destroy the PressureDriver object.
   * Performs any necessary cleanup before destroying the PressureDriver instance.
   */
  virtual ~PressureDriver();

  /**
   * @brief Retrieves the current pressure value.
   * This method returns the last successfully read pressure value. 
   * @return double The current pressure value.
   */
  double getPressure();

  /**
   * @brief Reads data from the pressure sensor. 
   * This method reads data from the pressure sensor and stores the response in the provided string.
   * @param[out] response A reference to a string where the sensor's response will be stored.
   * @return true if the data was successfully read, false otherwise.
   */
  bool read(std::string & response);

  /**< Maximum packet size for pressure data.*/
  static const int PRESSURE_MAX_PACKET_SIZE{32};

  /**< Buffer for storing the pressure data packet.*/
  uint8_t PRESSURE_PACKET[32];

private:
  /**< Boost.Asio IO service used for asynchronous I/O operations.*/
  boost::asio::io_service io_service_;

  /**< Pattern used for matching or extracting data from the sensor's response.*/
  std::string pattern_;

  /**
   * @brief Extracts a valid packet from the provided buffer.
   * This method processes the provided buffer and attempts to extract a valid data packet.
   * @param buffer A pointer to the buffer containing raw data.
   * @param buffer_size The size of the buffer.
   * @return int The number of bytes successfully extracted or a negative value on failure.
   */
  int extractPacket(const uint8_t * buffer, size_t buffer_size) const override;

  /**< Stores the last successfully read pressure value.*/
  mutable double pressure_value_ = 0.0;
};
}  // namespace pressure_pkg