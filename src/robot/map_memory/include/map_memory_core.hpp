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
    void initGlobalMap();
    bool shouldUpdateMap(double current_x, double current_y);
    void integrateCostmap();

    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  
    nav_msgs::msg::OccupancyGrid global_map_;

    double last_x_{0.0};
    double last_y_{0.0};
    bool is_first_odom_{true};
};

}  

#endif  
