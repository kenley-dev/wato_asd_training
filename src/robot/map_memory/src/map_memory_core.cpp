#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  latest_costmap_ = costmap;
}

void MapMemoryCore::updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odom) {
  latest_odom_ = odom;
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() {
  return global_map_;
}

} 
