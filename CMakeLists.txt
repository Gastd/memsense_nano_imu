cmake_minimum_required(VERSION 2.8.3)
project(memsense_nano_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES imu
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(imu_node src/imu_node.cpp src/memsense_nano_imu.cpp src/serialcom.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(imu_node
  ${catkin_LIBRARIES}
)
