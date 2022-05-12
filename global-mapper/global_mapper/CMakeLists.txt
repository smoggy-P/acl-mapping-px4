cmake_minimum_required(VERSION 2.8.3)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

#-------------------------------------------------------------------------------
# project initialization.
#-------------------------------------------------------------------------------
project(global_mapper)
set(${PROJECT_NAME}_VERSION 0.0.0)

# Options
option(BUILD_DOCUMENTATION "Use Doxygen to create the HTML based API documentation" OFF)
option(BUILD_TESTS "Use GTest to build and test libraries" OFF)

# Definitions
add_definitions("-std=c++14")
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# CMake Includes
include(cmake/setup.cmake)
include(cmake/install.cmake)

# External Dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common)

# setup global include directories
include_directories(SYSTEM
  src
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  )

#-------------------------------------------------------------------------------
# global_mapper library
#-------------------------------------------------------------------------------
# NOTE: You need to edit the config file (${PROJECT_NAME}Config.cmake.in) manually!
configure_file(${PROJECT_SOURCE_DIR}/cmake/templates/${PROJECT_NAME}Config.cmake.in
  "${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake" @ONLY)
configure_file(${PROJECT_SOURCE_DIR}/cmake/templates/${PROJECT_NAME}ConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)
install(FILES 
  ${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake 
  ${PROJECT_BINARY_DIR}/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake 
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME})

# global_mapper library
add_library(${PROJECT_NAME} SHARED
  src/global_mapper/global_mapper.cc
  )

target_link_libraries(${PROJECT_NAME}
  distance_grid
  cost_grid
  occupancy_grid
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# set_target_properties(${PROJECT_NAME} PROPERTIES 
#     COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY src/global_mapper DESTINATION include
  FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# occupancy_grid library
#-------------------------------------------------------------------------------
add_library(occupancy_grid SHARED
  src/occupancy_grid/occupancy_grid.cc
  )
install(TARGETS occupancy_grid
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY src/occupancy_grid DESTINATION include
  FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# distance_grid library
#-------------------------------------------------------------------------------
add_library(distance_grid SHARED
  src/distance_grid/distance_grid.cc
  )
install(TARGETS distance_grid
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY src/distance_grid DESTINATION include
  FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# cost_grid library
#-------------------------------------------------------------------------------
add_library(cost_grid SHARED
  src/cost_grid/cost_grid.cc
  )
target_link_libraries(cost_grid
  distance_grid
  occupancy_grid
  )
install(TARGETS cost_grid
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
install(DIRECTORY src/cost_grid DESTINATION include
  FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# voxel_grid library
#-------------------------------------------------------------------------------
install(DIRECTORY src/voxel_grid DESTINATION include
  FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# unit tests
#-------------------------------------------------------------------------------
add_executable(global_mapper_tests
  test/cost_grid_tests.cc
  test/voxel_grid_indexer_tests.cc
  test/distance_grid_tests.cc
  test/global_mapper_tests.cc
  test/gtest/gtest-all.cc
  )

target_link_libraries(global_mapper_tests
  pthread
  cost_grid
  distance_grid
  ${PROJECT_NAME}
  )

install(TARGETS global_mapper_tests
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  )
