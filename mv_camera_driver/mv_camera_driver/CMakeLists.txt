cmake_minimum_required(VERSION 3.10.0)
project(mv_camera_driver)

# set compilation
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-Wall -Wextra -Wno-unused-parameter -O2)
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/util.cmake)

# find dependencies
find_package(OpenCV REQUIRED) 
find_package(Threads REQUIRED) 

# ROS
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    )
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS 
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    )

# common include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    )

# find all source files
file(GLOB_RECURSE
    src_files
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*/*.cpp 
    )

# common library
add_library(
    ${PROJECT_NAME}
    SHARED
    ${src_files}
)
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    ${Boost_LIBS}
    Threads::Threads
    /lib/libMVSDK.so
    )

add_simple_apps() 