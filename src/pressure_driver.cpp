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

#include "pressure_pkg/pressure_driver.hpp"
#include <array>
#include <algorithm>

namespace pressure_pkg
{
PressureDriver::PressureDriver()
: ros_driver_base::Driver(PRESSURE_MAX_PACKET_SIZE), pattern_("*0001_")
{
  this->openURI("serial:///dev/ttyUSB0:9600");
  this->setReadTimeout(std::chrono::milliseconds(2000));
  this->setWriteTimeout(std::chrono::milliseconds(2000));
}

PressureDriver::~PressureDriver()
{
}

double PressureDriver::getPressure()
{
  std::string message;
  const uint8_t command[] = {'*', '0', '1', '0', '0', 'P', '4', '\r', '\n'};
  this->writePacket(command, sizeof(command));

  if (read(message)) {
    try {
      std::string cleaned_message = message;
      cleaned_message.erase(
        std::remove(
          cleaned_message.begin(),
          cleaned_message.end(), '\r'), cleaned_message.end());
      cleaned_message.erase(
        std::remove(
          cleaned_message.begin(),
          cleaned_message.end(), '\n'), cleaned_message.end());


      double pressure_value =
        std::stod(cleaned_message.substr(cleaned_message.find(pattern_) + pattern_.length()));
      return pressure_value;
    } catch (const std::invalid_argument & e) {
      std::cerr << "Erro ao converter a string para double: " << e.what() << std::endl;
    } catch (const std::out_of_range & e) {
      std::cerr << "Erro: valor fora do intervalo ao converter para double: " << e.what() <<
        std::endl;
    }
  }
  return 0.1;
}

bool PressureDriver::read(std::string & response)
{
  try {
    std::array<uint8_t, 32> message = {};
    int bytes_read = readPacket(message.data(), message.size());
    std::string read_response(reinterpret_cast<char const *>(message.data()), bytes_read);
    response = read_response;
    return true;
  } catch (const ros_driver_base::TimeoutError & e) {
    std::cerr << e.what() << '\n';
    return false;
  }
}

int PressureDriver::extractPacket(const uint8_t * buffer, size_t buffer_size) const
{
  std::string str(reinterpret_cast<const char *>(buffer));

  if (buffer[0] != '*') {
    return -buffer_size;
  }

  if (buffer_size < 18) {
    return 0;
  }

  size_t pos = str.find(pattern_);
  if (pos != std::string::npos) {
    std::string value_str = str.substr(pos + pattern_.length(), 10);
    pressure_value_ = std::stod(value_str);
    return buffer_size;
  }
  return -buffer_size;
}
}  // namespace pressure_pkg
