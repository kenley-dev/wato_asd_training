#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {
    initGlobalMap();
  }

void MapMemoryCore::initGlobalMap() {
  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = 0.1; // 0.1m per cell
  global_map_.info.width = 300;      // 300 cells wide (30m)
  global_map_.info.height = 300;     // 300 cells high (30m)

  // Center global grid origin at (-15m, -15m)
  global_map_.info.origin.position.x = -15.0;
  global_map_.info.origin.position.y = -15.0;
  global_map_.info.origin.position.z = 0.0;
  global_map_.info.origin.orientation.w = 1.0;

  // Initialize data with -1 (unknown space)
  global_map_.data.assign(global_map_.info.width * global_map_.info.height, -1);
}

void MapMemoryCore::updateCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap) {
  latest_costmap_ = costmap;
}

void MapMemoryCore::updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odom) {
  latest_odom_ = odom;

  if (!latest_costmap_) {
    return;
  }

  double current_x = odom->pose.pose.position.x;
  double current_y = odom->pose.pose.position.y;

  if (shouldUpdateMap(current_x, current_y)) {
    integrateCostmap();
  }

}

bool MapMemoryCore::shouldUpdateMap(double current_x, double current_y) {
  if (is_first_odom_) {
    last_x_ = current_x;
    last_y_ = current_y;
    is_first_odom_ = false;
    return true;
  }

  double dx = current_x - last_x_;
  double dy = current_y - last_y_;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance >= 1.5) { // Update if the robot moved more than 1.5 meters
    last_x_ = current_x;
    last_y_ = current_y;
    return true;
  }

  return false;
}

void MapMemoryCore::integrateCostmap() {
  if (!latest_costmap_ || !latest_odom_) return;

  // 1. Robot position and quaternion orientation
  double robot_x = latest_odom_->pose.pose.position.x;
  double robot_y = latest_odom_->pose.pose.position.y;

  double x = latest_odom_->pose.pose.orientation.x;
  double y = latest_odom_->pose.pose.orientation.y;
  double z = latest_odom_->pose.pose.orientation.z;
  double w = latest_odom_->pose.pose.orientation.w;

  // Convert Quaternion directly to Yaw using cmath (I had to do heavy research to find this formula and library)
  double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  // 2. Iterate through local costmap cells
  unsigned int costmap_w = latest_costmap_->info.width;
  unsigned int costmap_h = latest_costmap_->info.height;
  double costmap_res = latest_costmap_->info.resolution;
  double costmap_origin_x = latest_costmap_->info.origin.position.x;
  double costmap_origin_y = latest_costmap_->info.origin.position.y;

  for (unsigned int cy = 0; cy < costmap_h; ++cy) {
    for (unsigned int cx = 0; cx < costmap_w; ++cx) {
      int index = cy * costmap_w + cx;
      int8_t cost_val = latest_costmap_->data[index];

      // Skip unknown cells
      if (cost_val < 0) continue;

      // Cell position relative to local costmap frame
      double local_x = costmap_origin_x + (cx + 0.5) * costmap_res;
      double local_y = costmap_origin_y + (cy + 0.5) * costmap_res;

      // Transform local coordinate to global map frame
      double global_x = robot_x + (local_x * cos_yaw - local_y * sin_yaw);
      double global_y = robot_y + (local_x * sin_yaw + local_y * cos_yaw);

      // Map global coordinate into global grid indices
      int gx = std::floor((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
      int gy = std::floor((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

      // Bounds check and stitch cost into global grid
      if (gx >= 0 && gx < static_cast<int>(global_map_.info.width) &&
          gy >= 0 && gy < static_cast<int>(global_map_.info.height)) {
        int global_index = gy * global_map_.info.width + gx;
        global_map_.data[global_index] = cost_val;
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() {
  global_map_.header.stamp = rclcpp::Clock().now();
  return global_map_;
}

} 