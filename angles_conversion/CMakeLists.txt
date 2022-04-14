cmake_minimum_required(VERSION 2.8.3)
project(angles_conversion)

find_package(catkin REQUIRED COMPONENTS
  angles_conversion_srv
  rospy
)

catkin_package()

catkin_python_setup()
