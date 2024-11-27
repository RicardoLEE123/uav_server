cmake_minimum_required(VERSION 3.0.2)
project(uav_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  mavros_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp mavros mavros_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(uav_server src/uav_server.cpp)
target_link_libraries(uav_server  ${catkin_LIBRARIES}
)

## 添加安装规则
# install(TARGETS uav_server
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
