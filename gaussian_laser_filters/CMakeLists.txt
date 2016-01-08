cmake_minimum_required(VERSION 2.8.3)
project(gaussian_laser_filters)

SET(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  filters
  sensor_msgs
  std_msgs
  geometry_msgs
  pluginlib
  gslib
)

catkin_package(CATKIN_DEPENDS
  roscpp
  filters
  sensor_msgs
  std_msgs
  geometry_msgs
  pluginlib
  gslib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(beam_reverse_filter src/beam_reverse_filter.cc)
target_link_libraries(beam_reverse_filter ${catkin_LIBRARIES})

add_library(front_safe_filter src/front_safe_filter.cc)
target_link_libraries(front_safe_filter ${catkin_LIBRARIES})

add_library(angle_limit_filter src/angle_limit_filter.cc)
target_link_libraries(angle_limit_filter ${catkin_LIBRARIES})

add_library(lms1xx_filter src/lms1xx_filter.cc)
target_link_libraries(lms1xx_filter ${catkin_LIBRARIES})

add_library(single_laser_filter src/single_laser_filter.cc)
target_link_libraries(single_laser_filter ${catkin_LIBRARIES})
