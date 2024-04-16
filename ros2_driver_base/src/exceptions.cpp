#include <ros2_driver_base/exceptions.hpp>
#include <errno.h>
#include <string.h>

using namespace ros2_driver_base;

UnixError::UnixError(std::string const& desc)
    : std::runtime_error(desc + ": " + strerror(errno)), error(errno) {}

UnixError::UnixError(std::string const& desc, int error_code)
    : std::runtime_error(desc + ": " + strerror(error_code)), error(error_code) {}



