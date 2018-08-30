cmake_minimum_required(VERSION 2.8.3)
project(ball_tracker)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    image_transport
    rosconsole
    roscpp
    sensor_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(ball_tracker src/ball_tracker.cpp)
target_link_libraries(ball_tracker ${catkin_LIBRARIES})
