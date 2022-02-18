cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    )

find_package(OpenCV REQUIRED)
find_package(Eigen3)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
)

add_executable(KLT_EXPOSURE KLT_EXPOSURE.cpp)
target_link_libraries(KLT_EXPOSURE ${catkin_LIBRARIES} ${OpenCV_LIBS}  )
