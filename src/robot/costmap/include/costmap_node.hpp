#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void processParameters();

    // void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

    void publishMessage();
    void publishMessage2();
    // void publishCostmap(int **costmap_, const sensor_msgs::msg::LaserScan::SharedPtr scan);  <- OLD
 
  private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    // void convertToGrid(double range, double angle, double *x_local, double *y_local, const int MAP_RES);

    robot::CostmapCore costmap_;
    // Place these constructs here
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;

    std::string laserscan_topic_;
    std::string costmap_topic_;
  
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;

    double resolution_;
    int width_;
    int height_;
    geometry_msgs::msg::Pose origin_;
    double inflation_radius_;
};
 
#endif 