cmake_minimum_required(VERSION 3.0.2)
project(autonomous_exploration)

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


add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(detect_frontier src/detect_frontier.cpp)
target_link_libraries(detect_frontier ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(cluster_frontier src/cluster_frontier.cpp)
target_link_libraries(cluster_frontier ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(nbv_exploration src/nbv_exploration.cpp)
target_link_libraries(nbv_exploration ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})