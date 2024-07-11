cmake_minimum_required(VERSION 3.0.2)
project(pose_to_txt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_to_txt_node src/pose_to_txt_node.cpp)

target_link_libraries(pose_to_txt_node
  ${catkin_LIBRARIES}
)

install(TARGETS pose_to_txt_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

