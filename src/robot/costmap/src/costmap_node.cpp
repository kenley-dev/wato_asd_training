#include <chrono>
#include <memory>
#include "costmap_node.hpp"

using namespace std::chrono_literals;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar",10,
    std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1)); // Subscribe to the /lidar topic with a queue size of 10
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap",10); // Publish the costmap on the /costmap topic with a queue size of 10
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // pass the incoming LaserScan message to CostmapCore to process the 2D grid
  costmap_.updateCostmap(msg);
}

void CostmapNode::publishCostmap(){
  auto costmap_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();

  costmap_msg.header.stamp = this->now();
  costmap_msg.header.frame_id = "robot/base_link"; // or "base_link" depending on your TF frame

  costmap_msg.info.resolution = 0.1; // 0.1m per cell
  costmap_msg.info.width = 100;
  costmap_msg.info.height = 100;

  costmap_msg.info.origin.position.x = -5.0; // -(width * resolution / 2)
  costmap_msg.info.origin.position.y = -5.0; // -(height * resolution / 2)
  costmap_msg.info.origin.position.z = 0.0;

  const auto& 2d_grid = costmap_.getCostmap();
  costmap_msg.data.reserve(100 * 100);

  for (int j = 0; j < 100; ++j) {
    for (int i = 0; i < 100; ++i) {
      costmap_msg.data.push_back(static_cast<int8_t>(2d_grid[i][j]));
    }
  }

  costmap_pub_->publish(costmap_msg);
}


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}