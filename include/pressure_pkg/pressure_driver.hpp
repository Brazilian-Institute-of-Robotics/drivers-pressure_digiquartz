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
  explicit PressureDriver();
  virtual ~PressureDriver();
  
  // Método que envia o comando e retorna o valor da pressão
  double getPressure();
  float getTemperature();

  static const int PRESSURE_MAX_PACKET_SIZE{1024};
  uint8_t PRESSURE_PACKET[1024];
 
  // Método para extrair o valor da pressão do buffer

private:
  // Método auxiliar para converter o buffer em um valor de pressão
  std::string pattern_ = "*0001_";
  int extractPacket(const uint8_t* buffer, size_t buffer_size) const override;
  mutable double pressure_value_ = 0.0;
};

}