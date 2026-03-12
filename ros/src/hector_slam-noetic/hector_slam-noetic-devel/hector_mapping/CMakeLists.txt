cmake_minimum_required(VERSION 3.0.2)
project(hector_mapping)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  nav_msgs
  visualization_msgs
  tf
  tf2_ros                  # 新增
  message_filters
  laser_geometry
  message_generation
  std_srvs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  HectorDebugInfo.msg
  HectorIterData.msg
)

add_service_files(
  FILES
  ResetMapping.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    roscpp
    nav_msgs
    visualization_msgs
    tf
    tf2_ros                # 新增
    message_filters
    laser_geometry
    message_runtime
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(
  include/hector_slam_lib
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(hector_mapping
  src/HectorDebugInfoProvider.h
  src/HectorDrawings.h
  src/HectorMappingRos.h
  src/HectorMappingRos.cpp
  src/main.cpp
  src/PoseInfoContainer.cpp
  src/PoseInfoContainer.h
)

add_dependencies(hector_mapping hector_mapping_generate_messages_cpp)

target_link_libraries(hector_mapping
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS hector_mapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/hector_slam_lib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch/
)

## 安装原有 Python 脚本
catkin_install_python(PROGRAMS
  scripts/imu_serial_node.py
  scripts/serial_control.py
  scripts/path_follower_node.py
  # 新增：TF republisher 脚本
  scripts/odom_combined_to_base_footprint_republisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS
  scripts/serial_cmd_and_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
