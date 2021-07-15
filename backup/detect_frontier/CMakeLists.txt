cmake_minimum_required(VERSION 3.0.2)
project(detect_frontier)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs 
    genmsg 
    nav_msgs
    sensor_msgs
    cv_bridge
    image_transport
    actionlib
    actionlib_msgs
    move_base_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(detect_cluster_frontier src/detect_cluster_frontier.cpp)
target_link_libraries(detect_cluster_frontier ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(frontier_khs src/frontier.cpp)
target_link_libraries(frontier_khs ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
