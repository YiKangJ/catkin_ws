cmake_minimum_required(VERSION 2.8.3)

project(mrx_t4_a00_new)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    # tf
    # pcl_ros
    # roscpp
    # sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(CATKIN_DEPENDS
    #  tf
    # pcl_ros
    # roscpp
    # sensor_msgs

)

# add_executable(transform_node src/transform.cpp)

# target_link_libraries(transform_node ${catkin_LIBRARIES})

