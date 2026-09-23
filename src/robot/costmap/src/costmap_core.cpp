#include "costmap_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {

    costmap_ = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0)); // Initialize costmap with zeros

}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Clear the costmap
    for (auto& row : costmap_) {
        std::fill(row.begin(), row.end(), 0);
    }

    // Update the costmap based on the LaserScan data
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double angle = msg->angle_min + i * msg->angle_increment;
        double range = msg->ranges[i];

        if (range < msg->range_min || range > msg->range_max) {
            continue; // Ignore invalid ranges
        }

        // Convert polar coordinates to Cartesian coordinates
        int x = static_cast<int>((range * cos(angle)) / resolution_ + width_ / 2);
        int y = static_cast<int>((range * sin(angle)) / resolution_ + height_ / 2);

        // Check bounds and update the costmap
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            costmap_[x][y] = 1; // Mark as occupied
        }
    }

    angle += msg->angle_increment;

    }

    RCLCPP_INFO(logger_, "Updated costmap grid from LaserScan data.");

}