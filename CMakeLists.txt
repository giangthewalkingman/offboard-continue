cmake_minimum_required(VERSION 3.0.2)
project(offboard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslaunch
  # rospy
  std_msgs
  nav_msgs
  # mav_trajectory_generation 
  # mav_trajectory_generation_ros
  message_generation
  geometry_msgs
  roslib
)

find_package(JetsonGPIO)

# add_definitions(-std c++17)
set(CMAKE_CXX_STANDARD 17)

add_message_files(
  FILES
  PoseRequest.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES offboard
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime nav_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  include
  /usr/local/include
)

roslaunch_add_file_check(launch)

add_library(offboard
  src/offboard_lib.cpp
  src/pid_controller_base.cpp
)
target_link_libraries(offboard
  ${catkin_LIBRARIES}
  JetsonGPIO::JetsonGPIO
)
target_link_libraries(offboard
  /usr/lib/libwiringPi.so
)

############################################
# for build on your laptop
# target_link_libraries(offboard
#   /usr/lib/x86_64-linux-gnu/libcurses.so
#   /usr/lib/x86_64-linux-gnu/libncurses.so
#   /usr/lib/x86_64-linux-gnu/librt.so
# )

# for build on rpi server
target_link_libraries(offboard
  /usr/lib/aarch64-linux-gnu/libncurses.so
)

#############################################


add_executable(offboard_node src/offboard_node.cpp src/offboard_lib.cpp src/pid_controller_base.cpp)
target_link_libraries(offboard_node
  offboard
)

add_executable(setmode_offb src/setmode_offb.cpp)
target_link_libraries(setmode_offb
  ${catkin_LIBRARIES}
)

# catkin_install_python(PROGRAMS
#   scripts/MarkerDetection.py
#   scripts/real_cam.py
#   scripts/transform.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

