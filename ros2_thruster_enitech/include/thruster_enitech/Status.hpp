#ifndef THRUSTER_ENITECH_STATUS_HPP
#define THRUSTER_ENITECH_STATUS_HPP

#include <chrono>

namespace thruster_enitech
{
    /** Thruster status as reported by the PDO message */
    struct Status
    {
        /** The reception time of this status */
        std::chrono::system_clock::time_point time; // default UNIX in nanoseconds (uint64) 

        /** The thruster speed (rad/s) */
        double speed;
        /** The thruster current (A) */
        double current;

        /** Reported overtemperature on the electronics */
        bool overtemp_bg149;
        /** Reported overtemperature on the motor */
        bool overtemp_motor;

        /** Whether the extra gain for motor start is being used or not
         */
        bool start_gain;

        /** Whether the control parameters for air (true) or water (false) are
         * being used
         */
        bool air_parameters;

        enum CONTROL_MODE
         { RAW, SPEED }; //TODO: check protocol
        /** The control mode
         *
         * It is either RAW (current) or SPEED (rotation speed)
         */
        CONTROL_MODE control_mode;
    };
}

#endif

