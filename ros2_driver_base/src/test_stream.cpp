#include <ros2_driver_base/test_stream.hpp>
#include <ros2_driver_base/exceptions.hpp>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <chrono>

using namespace std;
using namespace ros2_driver_base;

/** Push data to the driver */
void TestStream::pushDataToDriver(vector<uint8_t> const& data)
{
    to_driver.insert(to_driver.end(), data.begin(), data.end());
}

/** Read all data that the device driver has written since the last
 * call to readDataFromDriver
 */
vector<uint8_t> TestStream::readDataFromDriver()
{
    vector<uint8_t> temp;
    temp.swap(from_driver);
    return temp;
}

void TestStream::waitRead(std::chrono::milliseconds const& timeout)
{
    if (to_driver.empty())
        throw TimeoutError(TimeoutError::NONE, "no data in to_device");
}
void TestStream::waitWrite(std::chrono::milliseconds const& timeout)
{
}

void TestStream::EXPECT_REPLY(std::vector<uint8_t> const& expectation, std::vector<uint8_t> const& reply)
{
    if(!mock_mode)
        throw MockContextException();
    expectations.push_back(expectation);
    replies.push_back(reply);
}


size_t TestStream::read(uint8_t* buffer, size_t buffer_size)
{
    size_t read_size = min(to_driver.size(), buffer_size);
    std::memcpy(buffer, to_driver.data(), read_size);
    to_driver.erase(to_driver.begin(), to_driver.begin() + read_size);
    return read_size;
}

size_t TestStream::write(uint8_t const* buffer, size_t buffer_size)
{
    if(mock_mode)
    {
        from_driver.insert(from_driver.end(), buffer, buffer + buffer_size);
        if(expectations.empty())
        {
            std::stringstream msg;
            msg << "Message received, but there are no expectations left:\n";
            for (std::vector<uint8_t>::const_iterator it = from_driver.begin(); it != from_driver.end(); ++it)
                msg << " " << setfill('0') << setw(2) << hex << static_cast<int>(*it);
            throw std::runtime_error(msg.str());
        }

        if(from_driver == expectations.front())
        {
            to_driver.insert(to_driver.end(), replies.front().begin(), replies.front().end());
            from_driver.clear();
            expectations.pop_front();
            replies.pop_front();
        }
        else
        {
            std::vector<uint8_t> const& expected = expectations.front();
            std::stringstream msg;
            msg << "ROS2_DRIVER_BASE_MOCK failure";
            msg << "\nExpected";
            for (std::vector<uint8_t>::const_iterator it = expected.begin(); it != expected.end(); ++it)
                msg << " " << setfill('0') << setw(2) << hex << static_cast<int>(*it);
            msg << "\nBut got ";
            for (std::vector<uint8_t>::const_iterator it = from_driver.begin(); it != from_driver.end(); ++it)
                msg << " " << setfill('0') << setw(2) << hex << static_cast<int>(*it);

            expectations.clear();
            replies.clear();
            throw std::invalid_argument(msg.str());
        }
        return buffer_size;
    }
    else
    {
        from_driver.insert(from_driver.end(), buffer, buffer + buffer_size);
        return buffer_size;
    }
}

void TestStream::clear()
{
    to_driver.clear();
}

void TestStream::clearExpectations()
{
    expectations.clear();
    replies.clear();
}


bool TestStream::expectationsAreEmpty()
{
    return(expectations.empty());
}

void TestStream::setMockMode(bool mode)
{
    mock_mode = mode;
    from_driver.clear();
}
