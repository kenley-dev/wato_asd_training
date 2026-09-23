#include <chrono>
#include <memory>
#include "costmap_node.hpp"

using namespace std::chrono_literals;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar",10,
    std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1)); // Subscribe to the /lidar topic with a queue size of 10
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received LaserScan message with %zu ranges", msg->ranges.size()); 
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}