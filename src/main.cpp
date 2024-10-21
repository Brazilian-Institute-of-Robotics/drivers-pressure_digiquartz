#include <stdexcept>
#include <string>
#include <iostream>
#include "pressure_pkg/pressure_driver.hpp"

int main(int argc, char **argv)
{
    std::ignore = argc;
    std::ignore = argv;

    // Inicializa o driver de pressão
    pressure_pkg::PressureDriver pressure_driver;

    double pressure_value{0.0};

    while (true)
    {
        try
        {
            pressure_value = pressure_driver.getPressure();
            std::cout << "Pressure: " << pressure_value << std::endl;
        }
        catch (const std::exception &error)
        {
            std::cout << error.what() << std::endl;
        }
    }

    return 0;
}