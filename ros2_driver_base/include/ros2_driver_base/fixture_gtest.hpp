#ifndef ROS2_DRIVER_BASE_GTEST_HPP
#define ROS2_DRIVER_BASE_GTEST_HPP

#include <gtest/gtest.h>
#include <ros2_driver_base/fixture.hpp>

#define ROS2_DRIVER_BASE_MOCK() ros2_driver_base::Fixture<fixture_driver_t>::GTestMockContext __context(this);

namespace ros2_driver_base {
    template<typename Driver>
    class Fixture<Driver>::GTestMockContext
    {
    public:
        Fixture* fixture;
        GTestMockContext(Fixture* fixture): fixture(fixture)
        {
            fixture->setMockMode(true);
        }
        /**
         * GTEST FAIL assertion can only be used in void-returning functions.
         * Constructors and Destructors are not considered void-returning functions,
         * according to the C++ language specification, and so you may not use fatal assertions in them.
         * Using a Fatal assertion on these method would leave the object in a partially state.
         * I was then decided to use the tear down to finalize the tests in GTEST.
         * In BOOST the teardown method can be normally called in the destructor method
         */
        void tearDown()
        {
            try
            {
                fixture->validateExpectationsAreEmpty();
            }
            catch(const TestEndsWithExpectationsLeftException& e)
            {
                ADD_FAILURE() << "ROS2_DRIVER_BASE_MOCK Error: Test reached its end without satisfying all expecations.";
            }
        }

        ~GTestMockContext()
        {
            fixture->setMockMode(false);
            fixture->clearExpectations();
            tearDown();
        }

    };
}

#endif
