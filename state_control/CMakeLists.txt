cmake_minimum_required(VERSION 2.8.3)
project(state_control_gp)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  controllers_manager
  std_msgs
  tf
  cmake_modules)
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    controllers_manager
    std_msgs 
    tf
  DEPENDS Eigen)

add_executable(state_control_gp src/state_control.cpp src/trajectory.cpp)
add_dependencies(state_control_gp ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_control_gp ${catkin_LIBRARIES})
