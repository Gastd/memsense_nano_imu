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

include(ExternalProject)
ExternalProject_Add(serialcom
  GIT_REPOSITORY https://github.com/Gastd/serialcom
  GIT_TAG master
  # PREFIX ${CMAKE_CURRENT_BINARY_DIR}/serialcom
  SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/serialcom-src"
  BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/serialcom-src/build"
  # CONFIGURE_COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" "${CMAKE_BINARY_DIR}/serialcom-src/"
  BUILD_COMMAND "${CMAKE_COMMAND}" --build .
  INSTALL_COMMAND ""
  TEST_COMMAND ""
)
# ExternalProject_Get_Property(serialcom install_dir)
ExternalProject_Get_Property(serialcom binary_dir)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/serialcom-src/include)
# message(${install_dir})

## Declare a C++ executable
add_executable(imu_node src/imu_node.cpp src/memsense_nano_imu.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(imu_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} serialcom)

## Specify libraries to link a library or executable target against
target_link_libraries(imu_node
  ${catkin_LIBRARIES}
  ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}serialcomlib.so
  -pthread
)
