cmake_minimum_required(VERSION 3.5)
project(ros2_canbus)

# Default to C99
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

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

find_package(drivers_canbus REQUIRED)


## Create customized msg and srv
set(msg_files
  "msg/ArrayCanbusMessage.msg"
  "msg/CanbusMessage.msg"
  "msg/CanbusStatistics.msg"
)

set(srv_files
  "srv/GetTopicName.srv"
  "srv/IsWatched.srv"
  "srv/Unwatch.srv"
  "srv/Watch.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES 
    builtin_interfaces std_msgs
  # ADD_LINTER_TESTS
 )

 # costumize messages
 ament_export_dependencies(rosidl_default_runtime)

 # https://github.com/ros2/common_interfaces/blob/980fc1858d104555f6f260e502c5ed96cd6b6e99/sensor_msgs/CMakeLists.txt#L54
# rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

#  if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

# set(THIS_PACKAGE_LIBS
#   src/Ros2Canbus.cpp
# )


# ## Declare a C++ library
# add_library(${PROJECT_NAME}_libs
# ${THIS_PACKAGE_LIBS}
# )

# ament_target_dependencies(${PROJECT_NAME}_libs
# drivers_canbus
# rclcpp
# rclcpp_lifecycle
# builtin_interfaces
# )

# target_include_directories(${PROJECT_NAME}_libs INTERFACE
#   PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
#   )

# target_link_libraries(${PROJECT_NAME}_lib INTERFACE
#   "${cpp_typesupport_target}")


# Declare C++ executable
add_executable(${PROJECT_NAME}_node src/Ros2Canbus.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  drivers_canbus
  rclcpp
  rclcpp_lifecycle
  builtin_interfaces
)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

## get build messages
# if Galactic and earlier
rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
# if Humble and later
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")


# target_link_libraries(${PROJECT_NAME}_node drivers_canbus)


# ## INSTALL directories

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
# install(
#   TARGETS ${PROJECT_NAME}_libs EXPORT export_${PROJECT_NAME}
# )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}_node
  DESTINATION
    lib/${PROJECT_NAME}
)

# # # Export old-style CMake variables
# # ament_export_include_directories("include/${PROJECT_NAME}")
# # # Export modern CMake targets
# # ament_export_targets(export_${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
