#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    const std::vector<std::vector<int>>& getCostmap() const;

  private:
    rclcpp::Logger logger_;

    // 2D costmap representation
    std::vector<std::vector<int>> costmap_;

    double resolution_ = 0.1; // metres per cell
    int width_ = 100; // number of cells in x-direction
    int height_ = 100; // number of cells in y-direction
};

}  

#endif  