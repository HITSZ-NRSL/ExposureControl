cmake_minimum_required(VERSION 2.8.3)
project(stander_tutorials)
add_definitions(-std=c++11 -Wall -Wextra)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport OpenCV rospy std_msgs genmsg sensor_msgs pcl_conversions pcl_ros)
find_package(Eigen3)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


include_directories("/usr/include/eigen3")
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(ExposureControl src/ExposureControl.cpp)
target_link_libraries(ExposureControl ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})
target_link_libraries(ExposureControl ${EIGEN3_LIBS})
add_dependencies(ExposureControl stander_tutorials_gencpp)