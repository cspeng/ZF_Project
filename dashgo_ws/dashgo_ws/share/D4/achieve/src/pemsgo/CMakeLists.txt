cmake_minimum_required(VERSION 2.8.3)
project(pemsgo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
#  LIBRARIES pemsgo
CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rviz_strip src/rviz_strip.cpp)

target_link_libraries(rviz_strip ${catkin_LIBRARIES})
