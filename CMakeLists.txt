cmake_minimum_required(VERSION 2.8.3)
project(imu_utils)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3  REQUIRED)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  "include"
)

catkin_package()



set(ACC_LIB_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/acc_lib/allan_acc.cpp
    ${PROJECT_SOURCE_DIR}/src/acc_lib/fitallan_acc.cpp
    )
set(GYR_LIB_SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/gyr_lib/allan_gyr.cpp
    ${PROJECT_SOURCE_DIR}/src/gyr_lib/fitallan_gyr.cpp
    )

add_executable(imu_an
    src/imu_an.cpp
    ${GYR_LIB_SOURCE_FILES}
    ${ACC_LIB_SOURCE_FILES}
    )

target_link_libraries(imu_an ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
