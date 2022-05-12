cmake_minimum_required(VERSION 3.0.2)
project(fla_utils)

find_package(catkin REQUIRED
  geometry_msgs
  sensor_msgs
  tf
  laser_geometry
  fla_msgs
  nodelet
  cv_bridge
  image_transport
  mavros_msgs)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_definitions("-std=c++11")

catkin_package(INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  )

add_executable(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
add_dependencies(laserscan_to_pointcloud fla_msgs_generate_messages_cpp)
target_link_libraries(laserscan_to_pointcloud ${catkin_LIBRARIES})

add_executable(quat_to_rpy src/quat_to_rpy.cpp)
add_dependencies(quat_to_rpy mavros_msgs_generate_messages_cpp)
target_link_libraries(quat_to_rpy ${catkin_LIBRARIES})

add_library(median_filter_nodelet src/median_filter_nodelet.cpp)
target_link_libraries(median_filter_nodelet
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

if(CATKIN_ENABLE_TESTING)
  # gtest
  enable_testing()

  catkin_add_gtest(${PROJECT_NAME}_test
    test/median_heap_test.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test median_filter_nodelet gtest ${catkin_LIBRARIES} ${GCOV_LIBRARY})
  target_compile_options(${PROJECT_NAME}_test PUBLIC ${${PROJECT_NAME}_COMPILER_FLAGS})
endif() # CATKIN_ENABLE_TESTING
