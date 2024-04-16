#ifndef ROS_UTILS_H
#define ROS_UTILS_H

// std libs
#include <iostream>
#include <chrono>

// ROS2 libs
#include <rclcpp/rclcpp.hpp>


namespace ros_utils {
    static inline std::chrono::system_clock::time_point rosTime2chronoTime(rclcpp::Time rosTime)
    {
        std::chrono::system_clock::time_point chronoTime{std::chrono::nanoseconds{rosTime.nanoseconds()}};
        return chronoTime;
    }

    static inline rclcpp::Time chronoTime2rosTime(std::chrono::system_clock::time_point chronoTime)
    {
        rclcpp::Time rosTime(chronoTime.time_since_epoch().count());
        return rosTime;
    }

    template <class T>
    static inline rclcpp::Duration durationInSeconds2rosDuration(T duration_in_seconds)
    {
        return rclcpp::Duration::Duration(std::chrono::duration<T>(duration_in_seconds));
    }

}


#endif
