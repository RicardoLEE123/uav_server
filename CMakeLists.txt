cmake_minimum_required(VERSION 3.0.2)
project(uav_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  mavros_msgs
  std_msgs
  tf
  common_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp mavros mavros_msgs std_msgs tf common_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(uav_server src/uav_server.cpp)
target_link_libraries(uav_server  ${catkin_LIBRARIES}
)

add_executable(camera_pose_tf src/camera_pose_tf.cpp)
target_link_libraries(camera_pose_tf  ${catkin_LIBRARIES}
)