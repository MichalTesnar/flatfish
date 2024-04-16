
cmake_minimum_required(VERSION 3.5)
project(ros2_driver_base)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(include)

add_library(ros2_driver_base SHARED
    src/driver.cpp
    src/bus.cpp
    src/timeout.cpp
    src/io_stream.cpp
    src/exceptions.cpp
    src/tcp_driver.cpp
    src/io_listener.cpp
    src/test_stream.cpp
)
ament_export_include_directories(include)
ament_export_targets(ros2_driver_base HAS_LIBRARY_TARGET)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

install(
    TARGETS ros2_driver_base
    EXPORT ros2_driver_base
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# TODO: ADD AGAIN
if(BUILD_TESTING)
    find_package(ament_cmake_gtest)
    find_package(Boost)
    enable_testing()
    ament_add_gtest(test_test_stream_gtest test/test_test_stream_gtest.cpp)
    target_link_libraries(test_test_stream_gtest ros2_driver_base ${Boost_LIBRARIES})

    # catkin does not have a function/macro for boost unit tests, work around this
    # ideally, we should migrate this to gtest
    find_package(Boost REQUIRED COMPONENTS unit_test_framework system)
    add_executable(test_Driver
        test/suite.cpp
        test/test_driver.cpp
        test/test_test_stream.cpp
    )
    target_compile_definitions(test_Driver PRIVATE BOOST_TEST_DYN_LINK)
    target_link_libraries(test_Driver ros2_driver_base ${Boost_LIBRARIES} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

    #add_dependencies(tests test_Driver)
    get_target_property(_target_path test_Driver RUNTIME_OUTPUT_DIRECTORY)

    add_test(NAME test_Driver
        COMMAND "${_target_path}/test_Driver" --log_format=XML
                                              --log_level=all
                                              --log_sink=${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}/boost-test_Driver.xml
        DEPENDENCIES test_Driver
        WORKING_DIRECTORY ${_target_path}
    )
    # end of adding boost test

    add_executable(test_tcp_read test/test_tcp_read.cpp)
    target_link_libraries(test_tcp_read ros2_driver_base ${Boost_LIBRARIES})

    add_executable(test_tcp_write test/test_tcp_write.cpp)
    target_link_libraries(test_tcp_write ros2_driver_base ${Boost_LIBRARIES})

    add_executable(test_udp_read test/test_udp_read.cpp)
    target_link_libraries(test_udp_read ros2_driver_base ${Boost_LIBRARIES})

    add_executable(test_udp_write test/test_udp_write.cpp)
    target_link_libraries(test_udp_write ros2_driver_base ${Boost_LIBRARIES})
endif()

ament_package()
