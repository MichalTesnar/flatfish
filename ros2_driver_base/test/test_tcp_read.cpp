#include <ros2_driver_base/driver.hpp>
#include <iostream>

using namespace ros2_driver_base;
using std::string;
using namespace std::literals::chrono_literals;

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
    driver.openURI(string("tcp://") + argv[1] + ":" + argv[2]);

    uint8_t buffer[10000];
    driver.setReadTimeout(std::chrono::milliseconds(1s));
    driver.readPacket(buffer, 10000);
    return 0;
}

