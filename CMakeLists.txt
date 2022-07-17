cmake_minimum_required(VERSION 3.0.2)
project(ezgo_control)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  std_msgs
  tablet_socket_msgs
  autoware_can_msgs
)

catkin_package(
   CATKIN_DEPENDS autoware_msgs roscpp std_msgs tablet_socket_msgs autoware_can_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arduino_ezgo_vehicle_control
  src/arduino_control.cpp
  src/SerialPort.cpp)
target_link_libraries(arduino_ezgo_vehicle_control ${catkin_LIBRARIES})
add_dependencies(arduino_ezgo_vehicle_control
  ${catkin_EXPORTED_TARGETS}
)

add_executable(ecu_ezgo_vehicle_control
  src/ecu_control.cpp)
target_link_libraries(ecu_ezgo_vehicle_control ${catkin_LIBRARIES})
if(EXISTS "/usr/include/canlib.h")
target_link_libraries(ecu_ezgo_vehicle_control canlib)
add_definitions(-DUSE_CANLIB)
message("'canlib is installed. Build kvaser.")
endif()
add_dependencies(ecu_ezgo_vehicle_control
  ${catkin_EXPORTED_TARGETS}
)
