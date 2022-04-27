cmake_minimum_required(VERSION 2.8.3)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

project(global_mapper_ros)

set(CMAKE_VERBOSE_MAKEFILE FALSE)

add_definitions("-std=c++11")
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# find catkin dependencies
set(REQ_CATKIN_PKGS   
  roscpp
  nodelet
  fla_utils
  pcl_ros
  pcl_msgs
  pcl_conversions
  tf2_ros
  nav_msgs
  visualization_msgs
  tf2_sensor_msgs
  message_generation
  image_transport
  cv_bridge
  fla_msgs
  snapstack_msgs
  )
find_package(catkin REQUIRED COMPONENTS ${REQ_CATKIN_PKGS})

# find system dependencies
find_package(PCL 1.7 REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(global_mapper REQUIRED)

add_message_files(FILES
  PlanningGrids.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
  )

# setup downstream dependencies
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${REQ_CATKIN_PKGS}
  DEPENDS PCL
  )

# setup global include directories
include_directories(
  src
  ${global_mapper_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

# global_mapper_node
add_executable(global_mapper_node
  src/global_mapper_node.cc
  )

# add node upstream dependencies
target_link_libraries(global_mapper_node
  ${catkin_LIBRARIES}
  global_mapper_ros
  )

# install node
install(TARGETS global_mapper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# global_mapper_nodelet
add_library(global_mapper_nodelet
  src/global_mapper_nodelet.cc
  )

# add nodelet upstream dependencies
target_link_libraries(global_mapper_nodelet
  ${catkin_LIBRARIES}
  global_mapper_ros
  )

# install nodelet
install(TARGETS global_mapper_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# global_mapper_ros wrapper library
add_library(global_mapper_ros
  src/global_mapper_ros/global_mapper_ros.cc
  )

target_link_libraries(global_mapper_ros
  ${global_mapper_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

add_dependencies(global_mapper_ros global_mapper_ros_generate_messages_cpp)

install(TARGETS global_mapper_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
