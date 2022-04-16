cmake_minimum_required(VERSION 2.8.3)
project(angles_conversion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  angles_conversion_srv
  rospy
)

catkin_package()

#catkin_python_setup()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(angles_conversion src/angles_conversion.cpp)
target_link_libraries(angles_conversion ${catkin_LIBRARIES})
add_dependencies(angles_conversion angles_conversion_generate_messages_cpp)

