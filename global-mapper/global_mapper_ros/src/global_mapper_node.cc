// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>

#include "ros/ros.h"

#include "global_mapper_ros/global_mapper_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_mapper_ros");

  global_mapper_ros::GlobalMapperRos global_mapper_ros;
  ROS_INFO("Starting loop");
  global_mapper_ros.Run();

  return 0;
}
