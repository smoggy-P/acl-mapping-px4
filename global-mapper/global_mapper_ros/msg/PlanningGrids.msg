Header header

float64[3] projected_goal  # current position of projected goal within the grid
float64[3] origin  # center of map in global frame [m]
int32[3] grid_dimensions  # size of map [voxels]
float64 resolution  # voxel size [m]

float32[] occupancy_data
int32[] distance_data
int32[] cost_data

float32 occupied_threshold #occupancy values above this are considered 'occupied'
int32 dmax  #max squared distance reported in distance_grid
