#include <digiquartz_pressure/Driver.hpp>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <boost/lexical_cast.hpp>

using namespace digiquartz_pressure;

Driver::Driver() 
    : iodrivers_base::Driver(1000000)
{
    buffer.resize(1000000);
    std::cout << "INIT abgeschlossen!" << std::endl;
}


int Driver::extractPacket (uint8_t const *buffer, size_t buffer_size) const
{
        for (int i = 0; i < buffer_size; i++){
            if (buffer[i] == '*'){
                if (i > 0)
                    std::cout << -i << std::endl;
                    return -i;
                
                for (int j = 2; j < 20; j++){
                    if (i+j >= buffer_size)
                        std::cout << 0 << std::endl;
                        return 0;

                    if (buffer[i+j-1] == 13 && buffer[i+j] == 10)
                        //std::cout << "size" << j+1 << std::endl;
                        std::cout << j << std::endl;
                        return j;
                }
            }
        }
        std::cout << -buffer_size << std::endl;
        return -buffer_size;
}

void Driver::open(std::string const& uri)
{
    std::cout << "URI:" << uri << std::endl;
    openURI(uri);
    //startAcquisition();
}

void Driver::startAcquisition()
{
    writePacket(reinterpret_cast<uint8_t const*>("*0100P4\n\r"), 9, 100);
}

bool Driver::readMeasurement(double &value)
{
    std::cout << "SIZE:" << buffer.size() << std::endl;
    base::Time timeout = base::Time().fromSeconds(10);
    int packet_size = readPacket(&buffer[0], buffer.size(), timeout);
    std::cout << "packet_size:" << packet_size << std::endl;
    std::cout << buffer[5] << std::endl;
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
    std::cout << "SIZE:" << buffer.size() << std::endl;
    base::Time timeout = base::Time().fromSeconds(3);
    int packet_size = readPacket(&buffer[0], buffer.size(), timeout);
    std::cout << "packet_size:" << packet_size << std::endl;
    std::cout << "buffer[7]:" << buffer[7] << std::endl;
    if (packet_size >= 10 && buffer[7] == '='){
        std::string param = std::string(reinterpret_cast<char const*>(&buffer[5]),2);
        std::cout << param << std::endl;     
        if ((cmd == PI && param == "PI")||
                (cmd == TI && param == "TI")||
                (cmd == PR && param == "PR")||
                (cmd == TR && param == "TR")||
                (cmd == OI && param == "OI")||
                (cmd == FM && param == "FM")||
                (cmd == UN && param == "UN"))
        {
            std::string s_value = std::string(reinterpret_cast<char const*>(&buffer[8]),packet_size-9);
            std::cout << s_value << std::endl;
            std::stringstream stream;
            stream << s_value;
            stream >> value;
            std::cout << "TTTTTTTTTTTTTTRRRRRRRRRRRRRRRRRRUUUUUUUUUUUUUUEEEEEEEEEEEEEEEEEEEE" << std::endl;
            return true;
        }
    }
    return false;
}

bool Driver::setConfig(Config &config){
    /*Config tmp = config;
    int t;
    std::stringstream cmd;
    cmd << "*0100EW*0100PI=" << config.pi;
    std::cout << cmd.str() << std::endl;
    writePacket(reinterpret_cast<uint8_t const*>(cmd.str().c_str()), cmd.str().length()+2, 100);
    readConfig(PI, t);
    std::cout << "T: " << t << std::endl;*/
    return true;
  //  writePacket(reinterpret_cast<uint8_t const*>("*0100EW*0100SU=1\n\r"), 18, 100);
  //  sleep(0.1);
  //  int test;
  //
  //  writePacket(reinterpret_cast<uint8_t const*>("*0100EW\n\r*0100UN=3\n\r"), 20, 200);
  //  this->readConfig(UN, test);
}
