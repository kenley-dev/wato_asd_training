#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);
    void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odom);

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  
    nav_msgs::msg::OccupancyGrid global_map_;
};

}  

#endif  
