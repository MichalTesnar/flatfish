#include <ros2_driver_base/driver.hpp>
#include <iostream>
#include <string.h>
using namespace std::literals::chrono_literals;

using namespace ros2_driver_base;
using std::string;

struct DisplayDriver : public ros2_driver_base::Driver
{
    DisplayDriver()
        : ros2_driver_base::Driver(10000) {}
    int extractPacket(uint8_t const* buffer, size_t size) const
    {
        std::cout << ros2_driver_base::Driver::printable_com(buffer, size) << std::endl;
        return -size;
    }
};

int main(int argc, char const* const* argv)
{
    DisplayDriver driver;
    driver.openURI(string("udpserver://") + argv[1]);

    uint8_t buffer[10000];
    driver.setReadTimeout(std::chrono::milliseconds(3s));
    driver.readPacket(buffer, 10000);
    return 0;
}

