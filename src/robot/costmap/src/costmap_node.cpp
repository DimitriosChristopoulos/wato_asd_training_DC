#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"

/*
Old laserCallback function, again I think the code is relatively correct but wasn't working very well
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Step 1: Initialize costmap
  const int MAP_RES = 10; 
  int** cost_map = initializeCostmap(MAP_RES);

  int inflation_radius = 1 * MAP_RES;

  // Step 2: Convert LaserScan to grid and mark obstacles
  for (int i = 0; i < scan->ranges.size(); i++){
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range <= scan->range_max && range >= scan->range_min){
      double x_grid, y_grid;

      convertToGrid(range, angle, &x_grid, &y_grid, MAP_RES);
      cost_map = markObstacleAndInflate(x_grid, y_grid, inflation_radius, cost_map);
    }
  }
  publishCostmap(cost_map, scan);
}

void CostmapNode::publishCostmap(int **costmap_, const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();

  occupancy_grid_msg.header = scan->header;
  // occupancy_grid_msg.header.frame_id = "map";


  // Fill the metadata (info)
  occupancy_grid_msg.info.resolution = 0.1;
  occupancy_grid_msg.info.width = 401;
  occupancy_grid_msg.info.height = 401;
  occupancy_grid_msg.info.origin.position.x = -20.0;
  occupancy_grid_msg.info.origin.position.y = -20.0;
  occupancy_grid_msg.info.origin.position.z = 0.0;
  occupancy_grid_msg.info.origin.orientation.w = 1.0;

  // Flatten the 2D costmap into a 1D array
  occupancy_grid_msg.data.reserve(401 * 401);
  for (size_t i = 0; i < 401; ++i){
    for (size_t j = 0; j < 401; ++j){
      occupancy_grid_msg.data.push_back(costmap_[j][i]);
    }
  }

  // RCLCPP_INFO(this->get_logger(), "Published costmap to /costmap");
  occupancy_grid_->publish(occupancy_grid_msg);
}
*/
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  processParameters();

  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  occ_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));

  costmap_.initCostmap(resolution_, width_, height_, origin_, inflation_radius_);
}

void CostmapNode::processParameters(){
  // Declare all ROS2 Parameters
  this->declare_parameter<std::string>("laserscan_topic", "/lidar");
  this->declare_parameter<std::string>("costmap_topic", "/costmap");
  this->declare_parameter<double>("costmap.resolution", 0.1);
  this->declare_parameter<int>("costmap.width", 400);
  this->declare_parameter<int>("costmap.height", 400);
  this->declare_parameter<double>("costmap.origin.position.x", -20.0);
  this->declare_parameter<double>("costmap.origin.position.y", -20.0);
  this->declare_parameter<double>("costmap.origin.orientation.w", 1.0);
  this->declare_parameter<double>("costmap.inflation_radius", 1.0);

  // Retrieve parameters and store them in member variables
  laserscan_topic_ = this->get_parameter("laserscan_topic").as_string();
  costmap_topic_ = this->get_parameter("costmap_topic").as_string();
  resolution_ = this->get_parameter("costmap.resolution").as_double();
  width_ = this->get_parameter("costmap.width").as_int();
  height_ = this->get_parameter("costmap.height").as_int();
  origin_.position.x = this->get_parameter("costmap.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("costmap.origin.position.y").as_double();
  origin_.orientation.w = this->get_parameter("costmap.origin.orientation.w").as_double();
  inflation_radius_ = this->get_parameter("costmap.inflation_radius").as_double();
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){
  costmap_.updateCostmap(scan);

  nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.getCostmapData();
  costmap_msg.header = scan->header;
  RCLCPP_INFO(this->get_logger(), "Publishing Costmap");
  occ_grid_pub_ ->publish(costmap_msg);
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2! TEST";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

void CostmapNode::publishMessage2() {
  auto message = std_msgs::msg::String();
  message.data = "TEST MSG";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}