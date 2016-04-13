

cmake_minimum_required(VERSION 2.8.3)
project(wc_chassis) 

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
  message(${Boost_INCLUDE_DIRS})
  message(${Boost_LIBRARY_DIRS})
  message(${Boost_LIBRARIES})
  message("boost found")
else()
  message("boost not found")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  autoscrubber_services
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf
    autoscrubber_services
)

set(GAUSSIAN_SDK_PATH "../sdk/")

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAUSSIAN_SDK_PATH}/include/
)

set(WC_CHASSIS_SRC
    ${GAUSSIAN_SDK_PATH}/src/Comm/Comm.cpp
    ${GAUSSIAN_SDK_PATH}/src/Comm/TimerDiff.cpp
    ${GAUSSIAN_SDK_PATH}/src/TransferDevice/SPort.cpp
    src/buffer.cpp
    src/protocol.cpp
    src/wc_chassis_mcu.cpp
    src/main.cpp
    src/init.cpp
    src/tools.cpp
    src/publish.cpp
)
add_executable(wc_chassis ${WC_CHASSIS_SRC})
target_link_libraries(wc_chassis ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
