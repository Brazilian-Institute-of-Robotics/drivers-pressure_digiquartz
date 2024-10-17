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

    
    double PressureDriver::sendCommand()
    {
        const uint8_t command[] = {'*', '0', '1', '0', '0', 'P', '4', '\r', '\n'};
        this->writePacket(command, sizeof(command));

        size_t size = this->readPacket(PRESSURE_PACKET, PRESSURE_MAX_PACKET_SIZE);

        if (extractPacket(PRESSURE_PACKET, size) == 1)
        {
            return pressure_value_;
        }
        else
        {
            return 0.0;
        }
    }

    bool PressureDriver::read(std::string & response)
    {
    std::vector<uint8_t> buffer;
    buffer.resize(kFinalPosition);

    try {
        int size = readPacket(&buffer[0], kFinalPosition);
        std::string raw = std::string(reinterpret_cast<char const *>(&buffer[0]), size);
        response = parseMessage(raw);
        return true;
    } catch (const ros_driver_base::TimeoutError & e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    return false;
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
        if (buffer[kInitPosition] != kInitMessage)
        {
            return -1;
        }

        if (buffer_size < kFinalPosition)
        {
            return 0;
        }                
        return -buffer_size;
    }

}