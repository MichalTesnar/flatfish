cmake_minimum_required(VERSION 3.5)
project(thruster_enitech)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drivers_canbus REQUIRED)
find_package(ros2_canbus REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(auv_control_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ThrusterStatus.msg"
  LIBRARY_NAME ${PROJECT_NAME}
  DEPENDENCIES std_msgs
)

# HACK FROM CHATGPT

# Assuming other necessary includes are already present
include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    "/home/michal/ros2_ws/src/ros2_canbus/include"  # Replace this path with the actual path to the include directory of ros2_canbus
)


add_library(${PROJECT_NAME}_lib
    src/NMT.cpp
    src/Protocol.cpp
    src/SDO.cpp
)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_include_directories(
  include
  )
ament_target_dependencies(${PROJECT_NAME}_lib
  drivers_canbus
  ros2_canbus
  auv_control_msgs)






add_executable(${PROJECT_NAME}_node src/ThrusterEnitechNode.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    rclcpp_lifecycle
    drivers_canbus
    ros2_canbus
    auv_control_msgs)

target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib)

rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
    TARGETS ${PROJECT_NAME}_lib
    EXPORT ${PROJECT_NAME}_lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)


install(
    TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)


# TODO gtest!

ament_package()
