// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <vector>
#include <string>

namespace global_mapper
{
struct Params
{
  std::string global_frame = "world";
  std::vector<double> origin = { 0, 0, 0 };
  std::vector<double> world_dimensions = { 5.0, 5.0, 1.0 };
  double resolution = 1.0;
  double radius_drone = 0.15;
  double Ra = 4.0;
  double z_ground = 0;
  int skip = 0;
  double depth_max = 10;
  double r1 = 0.8;
  double r2 = 8.0;
  double z_min_unknown = 0.2;
  double z_max_unknown = 5.0;

  // occupancy_grid
  double init_value = 0;
  double hit_inc = 0.2;
  double miss_inc = -0.01;
  double occupancy_threshold = 0.6;

  // distance_grid
  int truncation_distance = 6;  // in voxels

  // cost_grid
  int inflation_distance = 4;
  int altitude_weight = 20;
  int inflation_weight = 0;
  int unknown_weight = 20;
  int obstacle_weight = 10000;
};
}  // namespace global_mapper
