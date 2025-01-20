#include <algorithm>
#include <queue>

#include "costmap_core.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

/*
The below function was the first way I implemented marking obstacles in the cost map and inflating the obstacles. I'm pretty sure it
worked relatively well (obstacles appeared on the sim and looked somewhat correct), but I had alot of issues after finishing the
costmap node as a whole and implemented the update costmap and inflate obstacles functions from the answer key to continue moving
forward with the assignment, because I spent some time debugging and asked some WATO members for help and wasnt making progress on
fixing it

int** CostmapNode::markObstacleAndInflate(int x_grid, int y_grid, int inflation_radius, int** cost_map){
  const int MAX_COST = 25;
  cost_map[y_grid][x_grid] = 100; // setting cost of obstacle to high val

  for (int x_offset = -1*inflation_radius; x_offset < inflation_radius; x_offset++){
    for (int y_offset = -1*inflation_radius; y_offset < inflation_radius; y_offset++){
      double dist_to_point = sqrt((x_offset^2 + y_offset^2));
      if (cost_map[y_grid+y_offset][x_grid+x_offset] < MAX_COST){
        cost_map[y_grid+y_offset][x_grid+x_offset] = MAX_COST*(1 - dist_to_point / inflation_radius);
      }
    }
  }
  return cost_map;
}

Same for below function. After using the answer key as a reference I see that my x_grid and y_grid values needed to be calculated
with reference to the origin position of the robot.

void CostmapNode::convertToGrid(double range, double angle, double *x_grid, double *y_grid, const int MAP_RES){
  *x_grid = range*cos(angle);
  *y_grid = range*sin(angle);

  *x_grid = round(*x_grid * MAP_RES + 200);
  *y_grid = round(200 - *y_grid * MAP_RES);
}

Below is previous initCostmap() function. I was using a regular 2d array rather than a nav_msgs::msg::OccupancyGrid, using an
occupancy grid makes a lot more sense after implementing it that way, since that's what im going to publishing, no reason to
need to use a 2d array and then need to flatten it later.

int** CostmapNode::initializeCostmap(const int MAP_RES){
  const int ROWS = (20+20)*MAP_RES + 1; // 401 x 401 2d array because max range of lidar is 20, and robot will be at the middle of the 2d array
  const int COLS = (20+20)*MAP_RES + 1; // so even if the reading is 20 meters away, it will fit in the array with the robot being in the middle

  int** cost_map = new int*[COLS];
  for (int i = 0; i < COLS; i++){
    cost_map[i] = new int[ROWS];
    for (int j = 0; j < ROWS; j++){
      cost_map[i][j] = 0;
    }
  }
  return cost_map;
}
*/

void CostmapCore::initCostmap(double resolution, int width, int height, geometry_msgs::msg::Pose origin, double inflation_radius){
    if (!costmap_data_) 
        costmap_data_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    costmap_data_->info.resolution = resolution;
    costmap_data_->info.width = width;
    costmap_data_->info.height = height;
    costmap_data_->info.origin = origin;
    costmap_data_->data.assign(width * height, -1); // resizes "data" which is a vector to length width * height and fills it with -1

    inflation_radius_ = inflation_radius;

    inflation_cells_ = static_cast<int>(inflation_radius / resolution);

    RCLCPP_INFO(logger_, "Costmap initialized with resolution: %.2f, width: %d, height: %d", resolution, width, height);
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) const{
    std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), 0);

    for (int i = 0; i < scan->ranges.size(); i++){
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];

        if (range <= scan->range_max && range >= scan->range_min){
            double x_val = range*cos(angle);
            double y_val = range*sin(angle);

            int grid_x = static_cast<int>((x_val - costmap_data_->info.origin.position.x) / costmap_data_->info.resolution);
            int grid_y = static_cast<int>((y_val - costmap_data_->info.origin.position.y) / costmap_data_->info.resolution);

            if (grid_x > 0 && grid_x < costmap_data_->info.width){
                if (grid_y > 0 && grid_y < costmap_data_->info.height){
                    int index = grid_y * costmap_data_->info.height + grid_x;
                    costmap_data_->data[index] = 100;

                    inflateObstacle(grid_x, grid_y);
                }
            }
        }
    }
}


// This is obviously from the answer key, my tried implementing with previous version but couldnt get it to work,
// Implemented the answer key version to see why mine wasn't working and to move forward with the rest of the 
// Assignment
void CostmapCore::inflateObstacle(int grid_x, int grid_y) const{
  int need_to_write = 1;

  std::queue<std::pair<int, int>> queue;
  queue.emplace(grid_x, grid_y);

  std::vector<std::vector<bool>> visited(costmap_data_->info.width, std::vector<bool>(costmap_data_->info.height, false));
  visited[grid_x][grid_y] = true;

  while (!queue.empty()) {
    auto [x, y] = queue.front();
    queue.pop();

    // Iterate over neighboring cells
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;  // Skip the center cell

        int nx = x + dx;
        int ny = y + dy;

        // Ensure the neighbor cell is within bounds
        if (nx >= 0 && nx < static_cast<int>(costmap_data_->info.width) &&
          ny >= 0 && ny < static_cast<int>(costmap_data_->info.height) &&
          !visited[nx][ny]) {
          // Calculate the distance to the original obstacle cell
          double distance = std::hypot(nx - grid_x, ny - grid_y) * costmap_data_->info.resolution;

          // If within inflation radius, mark as inflated and add to queue
          if (distance <= inflation_radius_) {
              int index = ny * costmap_data_->info.width + nx;
              if (costmap_data_->data[index] < (1 - (distance / inflation_radius_)) * 100) {
                costmap_data_->data[index] = (1 - (distance / inflation_radius_)) * 100;
              }
              queue.emplace(nx, ny);
          }

          visited[nx][ny] = true;
        }
      }
    }
  }

}


nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const{
    return costmap_data_;
}

}