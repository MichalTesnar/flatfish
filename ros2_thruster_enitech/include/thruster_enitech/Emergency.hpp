#ifndef THRUSTER_ENITECH_EMERGENCY_HPP
#define THRUSTER_ENITECH_EMERGENCY_HPP

#include <chrono>

namespace thruster_enitech
{
    struct Emergency
    {
        std::chrono::system_clock::time_point time; // default UNIX in nanoseconds (uint64) 

        bool overtemp_motor;
        bool overtemp_bg149;

        bool fault_free;
        bool hardware_error;
        bool sensor_error;
        bool data_error;

        Emergency()
            : overtemp_motor(false)
            , overtemp_bg149(false)
            , fault_free(false)
            , hardware_error(false)
            , sensor_error(false)
            , data_error(false) {}
    };
}

#endif
