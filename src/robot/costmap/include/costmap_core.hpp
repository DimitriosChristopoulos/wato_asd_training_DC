#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initCostmap(double resolution, int width, int height, geometry_msgs::msg::Pose origin, double inflation_radius);
    // int** initializeCostmap(const int MAP_RES);  <- OLD, USED TO BE IN COSTMAP NODE.HPP

    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;
    // int** markObstacleAndInflate(int x_grid, int y_grid, int inflation_radius, int** cost_map);  <- OLD, USED TO BE IN COSTMAP_NODE.HPP

    void inflateObstacle(int origin_x, int origin_y) const;

    void publishMessage_2();

    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;

    double inflation_radius_;
    int inflation_cells_;

};

}  

#endif  