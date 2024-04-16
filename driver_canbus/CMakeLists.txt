cmake_minimum_required(VERSION 3.5)
project(drivers_canbus)

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

# check_include_files("sys/socket.h;linux/can.h" HAVE_CAN_H)
# if(HAVE_CAN_H)
#   message(STATUS "kernel support socket-can, building support for it")
#   list(APPEND CAN_SOCKET_SOURCES src/DriverSocket.cpp)
#   # list(APPEND CAN_SOCKET_HEADERS DriverSocket.hpp)
#   add_definitions(-DHAVE_CAN_H)
# else()
#   message(STATUS "kernel does not support socket-can")
# endif()

find_package(ament_cmake REQUIRED)
find_package(ros2_driver_base REQUIRED)

include_directories(
  include
  third_party)

add_library(${PROJECT_NAME}
  src/Driver2Web.cpp
  src/Driver.cpp
  src/DriverEasySYNC.cpp
  src/DriverHico.cpp
  src/DriverHicoPCI.cpp
  src/DriverNetGateway.cpp
  src/DriverVsCan.cpp
  src/DriverSocket.cpp
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(
  include
  third_party
  )
ament_target_dependencies(${PROJECT_NAME} ros2_driver_base)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ros2_driver_base)

install(
  DIRECTORY 
    include/${PROJECT_NAME}
    include/backwards
    third_party/vendor
  DESTINATION include
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)


add_executable(canbus-easysync src/tools/MainEasySYNC.cpp)
ament_target_dependencies(canbus-easysync ros2_driver_base)
target_link_libraries(canbus-easysync ${PROJECT_NAME})

add_executable(canbus-monitor src/tools/MainMonitor.cpp)
ament_target_dependencies(canbus-monitor ros2_driver_base)
target_link_libraries(canbus-monitor ${PROJECT_NAME})

add_executable(hico-tool src/tools/hcantool.c)

add_executable(canbus-reset src/tools/MainReset.cpp)
ament_target_dependencies(canbus-reset ros2_driver_base)
target_link_libraries(canbus-reset ${PROJECT_NAME})

add_executable(canbus-send src/tools/MainSend.cpp)
ament_target_dependencies(canbus-send ros2_driver_base)
target_link_libraries(canbus-send ${PROJECT_NAME})

install(
  TARGETS
    canbus-easysync
    canbus-monitor
    hico-tool
    canbus-reset
    canbus-send
  DESTINATION 
    tools
)

# TODO gtest!

ament_package()
