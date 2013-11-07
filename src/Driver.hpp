#ifndef DIGIQUARZ_PRESSURE_DRIVER_HPP
#define DIGIQUARZ_PRESSURE_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>

namespace digiquartz_pressure
{
    enum ConfigCmd{
        PI = 0,
        TI = 1,
        OI = 2,
        FM = 3,
        UN = 4
    };

    struct Config{
        int pi;
        int ti;
        int oi;
        int fm;
        int un;
    };


    class Driver : public iodrivers_base::Driver
    {
        std::vector<uint8_t> buffer;
        int extractPacket (uint8_t const *buffer, size_t buffer_size) const;

        private:
        bool readConfig(ConfigCmd cmd, int &value);
        
        public:

        Driver();

        /** Tries to access the Pressure Sensor at the provided URI
         */
        void open(std::string const& uri);

        /** Start acquisition
         *
         * The Devive get the command to send measured Pressure-Values.
         */
        void startAcquisition();

        /** Read available packets on the I/O */
        bool readMeasurement(double &value);

        /** Read the config from the device */
        Config getConfig();

        /** Sets the Integration Time for Pressure and Tempreture measurement 
         *  in milisecounds
         **/
        void setIntegrationTime(int integration_time);

        /** Read the current Integration Time for Pressure and Tempreture measurement
         *  from the divice.
         **/
        int getIntegrationTime();

        /** Read the integration mode from the Device*/
        int getOI();
    };
}

#endif
