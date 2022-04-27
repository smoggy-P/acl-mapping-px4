// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include "global_mapper_ros/global_mapper_ros.h"
#include "global_mapper/global_mapper.h"

namespace global_mapper_ros
{
class GlobalMapperNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit()
  {
    GlobalMapperRos global_mapper_ros;
    std::cout << "Starting LOOP*************" << std::endl;
    NODELET_INFO("*************************Starting loop");
    global_mapper_ros.Run();

    return;
  }
};

}  // namespace global_mapper_ros

// Export as a plugin.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(global_mapper_ros::GlobalMapperNodelet, nodelet::Nodelet)
