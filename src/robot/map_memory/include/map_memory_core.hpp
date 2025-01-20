#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initMap(){};
    void updateMap(){};
    void localToGlobal(){};

    nav_msgs::msg::OccupancyGrid::SharedPtr getMap();

  private:
    void integrateCostmap(){};

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    rclcpp::Logger logger_;
};

}  

#endif  
