#include <iostream>
#include <digiquartz_pressure/Driver.hpp>

int main(int argc, char** argv)
{
	digiquartz_pressure::Driver driver;
	driver.open(argv[1]);
        digiquartz_pressure::Config config;
        driver.setIntegrationTime(10);

        driver.startAcquisition();

        double value;
        while (driver.readMeasurement(value)){
            std::cout << value << std::endl;
        }
	return 0;
}
