#include "pressure_pkg/pressure_driver.hpp"
#include <string>
#include <iostream>
#include <vector>

using namespace boost::asio;
using namespace std;

namespace pressure_pkg
{
    PressureDriver::PressureDriver()
    : ros_driver_base::Driver(PRESSURE_MAX_PACKET_SIZE)
    {
        this->openURI("serial:///dev/ttyUSB0:9600");
        this->setReadTimeout(std::chrono::milliseconds(2000));
    }

    PressureDriver::~PressureDriver()
    {
    }

    double PressureDriver::getPressure()
    {
        const uint8_t command[] = {'*', '0', '1', '0', '0', 'P', '4', '\r', '\n'};
        this->writePacket(command, sizeof(command));

        size_t size = this->readPacket(PRESSURE_PACKET, PRESSURE_MAX_PACKET_SIZE);

        // Chama a função extractPacket e armazena o valor retornado
        if (extractPacket(PRESSURE_PACKET, size) == 1)
        {
            return pressure_value_;
        }
        else
        {
            return 0.0; // Retorna 0 se não conseguiu extrair um valor válido
        }
    }


    float PressureDriver::getTemperature()
    {
        const uint8_t command[] = {'*', '0', '1', '0', '0', 'Q', '4', '\r', '\n'};

        this->writePacket(command, sizeof(command));

        size_t size = this->readPacket(PRESSURE_PACKET, PRESSURE_MAX_PACKET_SIZE);

        return 0.0;
    }    

    int PressureDriver::extractPacket(const uint8_t *buffer, size_t buffer_size) const
    {
        // std::cout << "buffer value: " << buffer << std::endl;
        if (buffer[0] != '*')
        {
            return -1;
        }

        if (buffer_size < 18)
        {
            return 0;
        }

        std::string str(reinterpret_cast<const char*>(buffer));
        size_t pos = str.find(pattern_);
        std::cout << "pos: " << pos << std::endl;
        if (pos != std::string::npos) {
            std::string value_str = str.substr(pos + pattern_.length());
            std::cout << "Pressure: " << value_str << std::endl;
            pressure_value_ = std::stod(value_str);
            return 1;
        }
                
        return -buffer_size;
    }

}