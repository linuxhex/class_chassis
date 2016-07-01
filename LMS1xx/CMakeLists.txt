cmake_minimum_required(VERSION 2.8.3)
project(lms1xx)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC -O3 -DLASER_VERIFY_KEY=c636d97c70481dbbfbf9239f29ff3cbb")

# Build ROS-independent library.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})
add_library(LMS1xx STATIC src/LMS1xx.cpp)
target_link_libraries(LMS1xx ${console_bridge_LIBRARIES})

# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs common_services)
catkin_package(CATKIN_DEPENDS roscpp common_services)

include_directories(include ${catkin_INCLUDE_DIRS})
add_dependencies(LMS1xx ${catkin_EXPORTED_TARGETS})
add_executable(LMS1xx_node src/LMS1xx_node.cpp)
target_link_libraries(LMS1xx_node LMS1xx ${catkin_LIBRARIES})


install(TARGETS LMS1xx LMS1xx_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/find_sick scripts/set_sick_ip
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
