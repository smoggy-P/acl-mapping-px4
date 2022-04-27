// Copyright 2017 Massachusetts Institute of Technology

#include "occupancy_grid/occupancy_grid.h"

#include "global_mapper/global_mapper.h"

namespace global_mapper
{
GlobalMapper::GlobalMapper(Params& params)
  : params_(params)
  , occupancy_grid_(params_.origin.data(), params_.world_dimensions.data(), params_.resolution,
                    params_.occupancy_threshold)
  , distance_grid_(params_.origin.data(), params_.world_dimensions.data(), params_.resolution,
                   params_.truncation_distance)
  , cost_grid_(params_.origin.data(), params_.world_dimensions.data(), params_.resolution)
  , data_ready_(0)
{
  origin_[0] = 0.0;
  origin_[1] = 0.0;
  origin_[2] = 0.0;
  goal_[0] = 0.0;
  goal_[1] = 0.0;
  goal_[2] = 0.0;
  cost_grid_.SetDistanceGrid(&distance_grid_);
  cost_grid_.SetOccupancyGrid(&occupancy_grid_);
  cost_grid_.SetInflationDistance(params.inflation_distance);
  SetCostWeights(params_.altitude_weight, params_.inflation_weight, params_.unknown_weight, params_.obstacle_weight);
  occupancy_grid_.Reset(params.init_value);
}

GlobalMapper::~GlobalMapper()
{
  if (thread_.joinable())
  {
    thread_.join();
  }
}

void GlobalMapper::SetCostWeights(int altitude_weight, int inflation_weight, int unknown_weight, int obstacle_weight)
{
  cost_grid_.SetCostWeights(altitude_weight, inflation_weight, unknown_weight, obstacle_weight);
}

void GlobalMapper::SetObstacle(const double xyz[3])
{
  occupancy_grid_.UpdateValue(xyz, 1.0);
}

void GlobalMapper::RemoveObstacle(const double xyz[3])
{
  occupancy_grid_.UpdateValue(xyz, -1.0);
}

void GlobalMapper::UpdateGrids()
{
  UpdateDistanceGrid();
  UpdateCostGrid();
  occupancy_grid_.ResetDiffs();
}

void GlobalMapper::UpdateOrigin(const double xyz[3])
{
  std::lock_guard<std::mutex> origin_lock(origin_mutex_);
  memcpy(origin_, xyz, sizeof(double) * 3);
}

void GlobalMapper::PushPointCloud(const PointCloud::ConstPtr& cloud_ptr)
{
  // push
  std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);

  point_cloud_buffer_.clear();  // Jesus added this (remove all the previous point clouds--> queue will be only 1)
  point_cloud_buffer_.push_back(cloud_ptr);

  // notify
  std::unique_lock<std::mutex> data_lock(data_mutex_);
  data_ready_ = true;
  data_lock.unlock();
  condition_.notify_one();
}

const PointCloud::ConstPtr GlobalMapper::PopPointCloud()
{
  // pop
  std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
  PointCloud::ConstPtr cloud_ptr = nullptr;
  if (point_cloud_buffer_.size() > 0)
  {
    cloud_ptr = point_cloud_buffer_.front();
    point_cloud_buffer_.pop_front();
  }

  // notify
  std::unique_lock<std::mutex> data_lock(data_mutex_);
  if (point_cloud_buffer_.size() == 0)
  {
    data_ready_ = false;
  }

  return cloud_ptr;
}

void GlobalMapper::InsertPointCloud(const PointCloud::ConstPtr& cloud_ptr)
{
  if (!cloud_ptr)
  {
    return;
  }

  // insert point
  double start[3] = { cloud_ptr->sensor_origin_[0], cloud_ptr->sensor_origin_[1], cloud_ptr->sensor_origin_[2] };
  double end[3] = { 0.0 };

  int dim[3];
  dim[0] = params_.world_dimensions.data()[0];
  dim[1] = params_.world_dimensions.data()[1];
  dim[2] = params_.world_dimensions.data()[2];

  auto begin = std::chrono::steady_clock::now();

  for (int i = 0; i < cloud_ptr->points.size(); i++)
  {
    // clear
    end[0] = cloud_ptr->points[i].x;
    end[1] = cloud_ptr->points[i].y;
    end[2] = cloud_ptr->points[i].z;
    float intensity = cloud_ptr->points[i].intensity;
    // bool finite = std::isfinite(intensity);
    bool NaN = (intensity != intensity);

    if (NaN)
    {
      // NaN: clear unknown only up to clear_unknown_distance_
      occupancy_grid_.RayTrace(start, end, 0);
    }
    else
    {
      // Inf and valid: clear occupied and unknown
      // --->Inf: Up to clear_unknown_distance_ (value saved in point.z)
      // --->Valid: Up to the finite value saved in point.z.
      occupancy_grid_.RayTrace(start, end, params_.miss_inc);
    }
  }

  auto end_time = std::chrono::steady_clock::now();
  auto diff = end_time - begin;
  // std::cout << "1st loop:  " << std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() << " ms "
  //           << std::endl;

  auto begin2 = std::chrono::steady_clock::now();

  for (int i = 0; i < cloud_ptr->points.size(); i++)
  {
    float intensity = cloud_ptr->points[i].intensity;
    if (std::isfinite(intensity))
    {
      end[0] = cloud_ptr->points[i].x;
      end[1] = cloud_ptr->points[i].y;
      end[2] = cloud_ptr->points[i].z;
      occupancy_grid_.UpdateValue(end, params_.hit_inc);
    }
  }

  auto end_time2 = std::chrono::steady_clock::now();
  auto diff2 = end_time2 - begin2;
  // std::cout << "2nd loop:  " << std::chrono::duration_cast<std::chrono::milliseconds>(diff2).count() << " ms "
  //           << std::endl;
}

void GlobalMapper::GetVoxelGrids(voxel_grid::VoxelGrid<float>* occupancy_grid,
                                 voxel_grid::VoxelGrid<int>* distance_grid, voxel_grid::VoxelGrid<int>* cost_grid)
{
  std::lock_guard<std::mutex> output_lock(output_mutex_);
  *occupancy_grid = occupancy_grid_;
  *distance_grid = distance_grid_;
  *cost_grid = cost_grid_;
}

void GlobalMapper::UpdateOccupancyGrid()
{
  PointCloud::ConstPtr cloud_ptr = PopPointCloud();
  InsertPointCloud(cloud_ptr);

  // printf("RadiusDrone=%f\n", params_.radius_drone);
  // int n = (params_.radius_drone) / (params_.resolution);  // Number of voxels to clear in each side
  // n = (n > 1) ? n : 1;                                    // force n to be at least 1

  int n = 1;
  // clear voxels around vehicle
  int origin_ixyz[3];
  occupancy_grid_.WorldToGrid(origin_, origin_ixyz);
  for (int i = -n; i <= n; i++)
  {
    for (int j = -n; j <= n; j++)
    {
      for (int k = -n; k <= n; k++)
      {
        // printf("Clearing cell around vehicle!");
        int ixyz[3];
        ixyz[0] = origin_ixyz[0] + i;
        ixyz[1] = origin_ixyz[1] + j;
        ixyz[2] = origin_ixyz[2] + k;
        occupancy_grid_.UpdateValue(ixyz, -1.0);
      }
    }
  }
}

void GlobalMapper::UpdateCostGrid()
{
  cost_grid_.UpdateDijkstra(goal_);
  dense_path_.clear();
  sparse_path_.clear();
  cost_grid_.GetPaths(origin_, &dense_path_, &sparse_path_);
}

void GlobalMapper::GetPaths(std::vector<std::array<double, 3>>* dense_path,
                            std::vector<std::array<double, 3>>* sparse_path)
{
  std::lock_guard<std::mutex> output_lock(output_mutex_);
  *dense_path = dense_path_;
  *sparse_path = sparse_path_;
}

void GlobalMapper::UpdateDistanceGrid()
{
  for (auto const& cleared_point : occupancy_grid_.GetCleared())
  {
    distance_grid_.RemoveObstacle(cleared_point.data());
  }
  distance_grid_.UpdateDistances();

  for (auto const& marked_point : occupancy_grid_.GetMarked())
  {
    distance_grid_.SetObstacle(marked_point.data());
  }
  distance_grid_.UpdateDistances();
}

void GlobalMapper::GetOrigin(double xyz[3]) const
{
  memcpy(xyz, origin_, sizeof(double) * 3);
}

bool GlobalMapper::GetProjectedGoal(double xyz[3]) const
{
  return cost_grid_.GetProjectedGoal(xyz);
}

void GlobalMapper::SetGoal(const double xyz[3])
{
  memcpy(goal_, xyz, sizeof(double) * 3);
}

void GlobalMapper::GetGoal(double xyz[3]) const
{
  memcpy(xyz, goal_, sizeof(double) * 3);
}

void GlobalMapper::Spin()
{
  static int spincount = 0;

  while (true)
  {
    std::unique_lock<std::mutex> data_lock(data_mutex_);
    condition_.wait(data_lock, [this] { return data_ready_; });
    data_lock.unlock();

    std::unique_lock<std::mutex> output_lock(output_mutex_);

    origin_mutex_.lock();
    occupancy_grid_.UpdateOrigin(origin_);
    // distance_grid_.UpdateOrigin(origin_);
    // cost_grid_.UpdateOrigin(origin_);
    origin_mutex_.unlock();

    occupancy_grid_.ResetDiffs();
    UpdateOccupancyGrid();
    // UpdateDistanceGrid();
    // if ((spincount++ % 15) == 0)
    //{
    //  UpdateCostGrid();
    //}
    output_lock.unlock();
  }
}

void GlobalMapper::Run()
{
  fprintf(stderr, "GlobalMapper::Run\n");

  thread_ = std::thread(&GlobalMapper::Spin, this);
}

}  // namespace global_mapper
