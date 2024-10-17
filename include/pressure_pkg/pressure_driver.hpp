#pragma once

#include <string>
#include <iostream>

#include <boost/asio.hpp>
#include "ros_driver_base/driver.hpp"
#include "pressure_pkg/visibility_control.h"

using namespace boost::asio;
using namespace std;

namespace pressure_pkg
{

class PressureDriver : public ros_driver_base::Driver
{
public:
  /**<Enum for buffer indices. */
  enum BufferIndices
  {
    kInitPosition = 0,
    kFinalPosition = 18
  };
  explicit PressureDriver();
  virtual ~PressureDriver();
  
  double getPressure();
  float getTemperature();

  static const int PRESSURE_MAX_PACKET_SIZE{1024};
  uint8_t PRESSURE_PACKET[1024];
 

private:
  std::string pattern_ = "*0001_";
  std::string kInitMessage = '*';
  bool PressureDriver::read(std::string & response);
  int extractPacket(const uint8_t* buffer, size_t buffer_size) const override;
  mutable double pressure_value_ = 0.0;
};

}