#include <digiquartz_pressure/Driver.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

using namespace digiquartz_pressure;

Driver::Driver() 
    : iodrivers_base::Driver(1000000)
{
    buffer.resize(1000000);
}


int Driver::extractPacket (uint8_t const *buffer, size_t buffer_size) const
{
        for (int i = 0; i < buffer_size; i++){
            if (buffer[i] == '*'){
                if (i > 0)
                    return -i;
                
                for (int j = 2; j < 20; j++){
                    if (i+j >= buffer_size)
                        return 0;

                    if (buffer[i+j-1] == 13 && buffer[i+j] == 10)
                        return j;
                }
            }
        }
        return -buffer_size;
}

void Driver::open(std::string const& uri)
{
    openURI(uri);
}

void Driver::startAcquisition()
{
    // set a fixed field size for the measurement
    std::string set_fixed_field = "*0100EW*0100DL=1\n\r";
    writePacket(reinterpret_cast<uint8_t const*>(set_fixed_field.data()), set_fixed_field.size(), 100);
    usleep(100000);
    // add a separator character to the measurement
    std::string add_separator = "*0100EW*0100SU=1\n\r";
    writePacket(reinterpret_cast<uint8_t const*>(add_separator.data()), add_separator.size(), 100);
    usleep(100000);
    // start continuous measurement
    writePacket(reinterpret_cast<uint8_t const*>("*0100P4\n\r"), 9, 100);
}

bool Driver::readMeasurement(double &value)
{
    base::Time timeout = base::Time().fromSeconds(3);
    int packet_size = readPacket(&buffer[0], buffer.size(), timeout);
    if (packet_size == 18 && buffer[5] == '_'){
        std::string s_value = std::string(reinterpret_cast<char const*>(&buffer[6]), 10);
        std::stringstream stream;
        stream << s_value;
        stream >> value;
        return true;
    } 
    return false;

}

bool Driver::readConfig(ConfigCmd cmd, int &value)
{
    base::Time timeout = base::Time().fromSeconds(1);
    int packet_size = readPacket(&buffer[0], buffer.size(), timeout);
    if (packet_size >= 10 && buffer[7] == '='){
        std::string param = std::string(reinterpret_cast<char const*>(&buffer[5]),2);
        if ((cmd == PI && param == "PI")||
                (cmd == TI && param == "TI")||
                (cmd == OI && param == "OI")||
                (cmd == FM && param == "FM")||
                (cmd == UN && param == "UN"))
        {
            std::string s_value = std::string(reinterpret_cast<char const*>(&buffer[8]),packet_size-9);
            std::stringstream stream;
            stream << s_value;
            stream >> value;
            return true;
        }
    }
    return false;
}

void Driver::setIntegrationTime(int integration_time){
    if (getOI())
        integration_time = integration_time/2;
    std::stringstream cmd;
    cmd << "*0100EW*0100PI=" << integration_time << "\n\r";
    writePacket(reinterpret_cast<uint8_t const*>(cmd.str().c_str()), cmd.str().length()+2, 100);
    while(!readConfig(PI, integration_time));

}

int Driver::getIntegrationTime(){
    int pi;

    std::stringstream cmd;
    cmd << "*0100EW*0100PI\n\r";
    writePacket(reinterpret_cast<uint8_t const*>(cmd.str().c_str()), cmd.str().length()+2, 100);
    while(!readConfig(PI, pi));

    if (getOI())
        pi = pi*2;

    return pi;
}

int Driver::getOI(){
    int oi;

    std::stringstream cmd;
    cmd << "*0100EW*0100OI\n\r";
    writePacket(reinterpret_cast<uint8_t const*>(cmd.str().c_str()), cmd.str().length()+2, 100);
    while(!readConfig(OI, oi));

    return oi;
}


Config Driver::getConfig(){
    Config config;
    
    std::stringstream cmd;
    cmd << "*0100EW*0100PI\n\r";
    writePacket(reinterpret_cast<uint8_t const*>(cmd.str().c_str()), cmd.str().length()+2, 100);
    while(!readConfig(PI, config.pi));

    cmd.str("");
    cmd << "*0100EW*0100TI\n\r";
    writePacket(reinterpret_cast<uint8_t const*>(cmd.str().c_str()), cmd.str().length()+2, 100);
    while(!readConfig(TI, config.ti));
    
    cmd.str("");
    cmd << "*0100EW*0100OI\n\r"; 
    writePacket(reinterpret_cast<uint8_t const*>(cmd.str().c_str()), cmd.str().length()+2, 100);
    while(!readConfig(OI, config.oi));

    cmd.str("");
    cmd << "*0100EW*0100FM\n\r";
    writePacket(reinterpret_cast<uint8_t const*>(cmd.str().c_str()), cmd.str().length()+2, 100);
    while(!readConfig(FM, config.fm));
    
    cmd.str("");
    cmd << "*0100EW*0100UN\n\r";
    writePacket(reinterpret_cast<uint8_t const*>(cmd.str().c_str()), cmd.str().length()+2, 100);
    while(!readConfig(UN, config.un));
    
    return config;
}
