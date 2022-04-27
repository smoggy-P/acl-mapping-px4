// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <snapstack_msgs/State.h>

#include "global_mapper/global_mapper.h"
#include "global_mapper_ros/PlanningGrids.h"

namespace global_mapper_ros
{
class GlobalMapperRos
{
public:
  GlobalMapperRos();
  void Run();

private:
  void GetParams();
  void InitSubscribers();
  void InitPublishers();
  void PopulateUnknownPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                    sensor_msgs::PointCloud2* pointcloud);
  void PopulateOccupancyPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                      sensor_msgs::PointCloud2* pointcloud);
  void PopulateDistancePointCloudMsg(const voxel_grid::VoxelGrid<int>& distance_grid,
                                     sensor_msgs::PointCloud2* pointcloud);
  void PopulateCostPointCloudMsg(const voxel_grid::VoxelGrid<int>& cost_grid, sensor_msgs::PointCloud2* pointcloud);
  void PopulatePathMsg(const std::vector<std::array<double, 3>>& path, nav_msgs::Path* path_msg);
  void Publish(const ros::TimerEvent& event);

  void PublishPlanningGrids(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                            const voxel_grid::VoxelGrid<int>& distance_grid,
                            const voxel_grid::VoxelGrid<int>& cost_grid);

  // callbacks
  void DepthImageCallback(const sensor_msgs::Image::ConstPtr& image,
                          const sensor_msgs::CameraInfo::ConstPtr& camera_info);
  void PoseCallback(const geometry_msgs::PoseStamped& pose_ptr);
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_ptr);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr);

  // health and status
  enum ProcessArgs
  {
    NOMINAL = 0,
    NO_POSE = 1,
    NO_GOAL = 2,
    NO_DEPTH_IMAGE = 3
  };

  // name of the drone
  std::string name_drone;

  // publishers
  ros::Publisher occ_grid_pub_;
  ros::Publisher unknown_grid_pub_;
  ros::Publisher frontier_grid_pub_;
  ros::Publisher dist_grid_pub_;
  ros::Publisher cost_grid_pub_;
  ros::Publisher path_pub_;
  ros::Publisher sparse_path_pub_;
  ros::Publisher planning_grids_pub_;

  // time (in secs) of the last point cloud fused in this map
  ros::Time tstampLastPclFused_;

  ros::Timer grid_pub_timer_;

  // subscribers
  image_transport::CameraSubscriber depth_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;

  // params
  global_mapper::Params params_;
  bool publish_occupancy_grid_;
  bool publish_unknown_grid_;
  bool publish_distance_grid_;
  bool publish_cost_grid_;
  bool publish_path_;
  double clear_unknown_distance_;
  double target_altitude_;

  // i/o flags
  bool got_goal_;
  bool got_pose_;
  bool got_depth_image_;

  geometry_msgs::PoseStamped uav_pose_smj;

  std::unique_ptr<global_mapper::GlobalMapper> global_mapper_ptr_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<image_transport::ImageTransport> it_ptr_;
};
}  // namespace global_mapper_ros
