#include "map_memory_node.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"), 
    map_memory_(robot::MapMemoryCore(this->get_logger())) 
{

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    // Set up timer (e.g. update/publish every 1 second)
  timer_ = this->create_wall_timer(
      1s, std::bind(&MapMemoryNode::updateMap, this)
  );
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  map_memory_.updateCostmap(msg);
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  map_memory_.updateOdometry(msg);
}

void MapMemoryNode::updateMap() {
  nav_msgs::msg::OccupancyGrid global_map = map_memory_.getGlobalMap();
  map_pub_->publish(global_map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
