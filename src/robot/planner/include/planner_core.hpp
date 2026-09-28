#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace robot 
{
  enum class PlannerState {
    WAITING_FOR_GOAL,
    REACHING_GOAL
  };

  class PlannerCore {
    public:
      explicit PlannerCore(const rclcpp::Logger& logger);

      void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
      void updateGoal(const geometry_msgs::msg::PointStamped::SharedPtr goal);
      void updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odom);

      bool shouldReplan();
      nav_msgs::msg::Path planPath();

    private:
      rclcpp::Logger logger_;

      nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
      geometry_msgs::msg::PointStamped::SharedPtr current_goal_;
      nav_msgs::msg::Odometry::SharedPtr current_odom_;

      PlannerState state_{PlannerState::WAITING_FOR_GOAL};
      bool goal_received_{false};
  };
}  

#endif  
