#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" // Include the LaserScan message type
#include "nav_msgs/msg/occupancy_grid.hpp" // Include the OccupancyGrid message type for publishing the costmap
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishCostmap();

  robot::CostmapCore costmap_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

#endif