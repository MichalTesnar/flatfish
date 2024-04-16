#ifndef ROS2_DRIVER_BASE_BOOST_HPP
#define ROS2_DRIVER_BASE_BOOST_HPP

#include <boost/test/unit_test.hpp>
#include <ros2_driver_base/fixture.hpp>

#define ROS2_DRIVER_BASE_MOCK() ros2_driver_base::Fixture<fixture_driver_t>::BoostMockContext __context(this);

namespace ros2_driver_base {
    template<typename Driver>
    class Fixture<Driver>::BoostMockContext
    {
    public:
        Fixture* fixture;
        BoostMockContext(Fixture* fixture): fixture(fixture)
        {
            fixture->setMockMode(true);
        }

        void tearDown()
        {
            try
            {
                fixture->validateExpectationsAreEmpty();
            }
            catch(const TestEndsWithExpectationsLeftException& e)
            {
                BOOST_ERROR("ROS2_DRIVER_BASE_MOCK Error: Test reached its end without satisfying all expecations.");
            }
        }

        ~BoostMockContext()
        {
            tearDown();
            fixture->setMockMode(false);
        }

    };
}

#endif
