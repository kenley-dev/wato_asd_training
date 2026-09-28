#include "planner_core.hpp"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>
#include <algorithm>

namespace robot
{

struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const {
    return (x == other.x && y == other.y);
  }
  bool operator!=(const CellIndex &other) const {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode {
  CellIndex index;
  double f_score;

  bool operator>(const AStarNode &other) const {
    return f_score > other.f_score;
  }
};

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

void PlannerCore::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  current_map_ = map;
}

void PlannerCore::updateGoal(const geometry_msgs::msg::PointStamped::SharedPtr goal) {
  current_goal_ = goal;
  goal_received_ = true;
  state_ = PlannerState::REACHING_GOAL;
  RCLCPP_INFO(logger_, "New goal received: (%f, %f)", goal->point.x, goal->point.y);
}

void PlannerCore::updateOdometry(const nav_msgs::msg::Odometry::SharedPtr odom) {
  current_odom_ = odom;
}

bool PlannerCore::shouldReplan() {
  if (state_ == PlannerState::REACHING_GOAL && current_map_ && current_goal_ && current_odom_) {
    double robot_x = current_odom_->pose.pose.position.x;
    double robot_y = current_odom_->pose.pose.position.y;
    double goal_x = current_goal_->point.x;
    double goal_y = current_goal_->point.y;

    double dist = std::hypot(goal_x - robot_x, goal_y - robot_y);
    if (dist < 0.5) {
      RCLCPP_INFO(logger_, "Goal reached!");
      state_ = PlannerState::WAITING_FOR_GOAL;
      return false;
    }
    return true;
  }
  return false;
}

CellIndex worldToGrid(double wx, double wy, const nav_msgs::msg::OccupancyGrid &map) {
  int gx = std::floor((wx - map.info.origin.position.x) / map.info.resolution);
  int gy = std::floor((wy - map.info.origin.position.y) / map.info.resolution);
  return CellIndex(gx, gy);
}

void gridToWorld(const CellIndex &idx, const nav_msgs::msg::OccupancyGrid &map, double &wx, double &wy) {
  wx = map.info.origin.position.x + (idx.x + 0.5) * map.info.resolution;
  wy = map.info.origin.position.y + (idx.y + 0.5) * map.info.resolution;
}

nav_msgs::msg::Path PlannerCore::planPath() {
  nav_msgs::msg::Path path;

  if (!current_map_ || !current_goal_ || !current_odom_) {
    return path;
  }

  path.header.frame_id = current_map_->header.frame_id;
  path.header.stamp = rclcpp::Clock().now();

  CellIndex start = worldToGrid(current_odom_->pose.pose.position.x, current_odom_->pose.pose.position.y, *current_map_);
  CellIndex goal = worldToGrid(current_goal_->point.x, current_goal_->point.y, *current_map_);

  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  auto heuristic = [](const CellIndex &a, const CellIndex &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
  };

  open_set.push({start, heuristic(start, goal)});
  g_score[start] = 0.0;

  int dx[] = {1, -1, 0, 0, 1, 1, -1, -1};
  int dy[] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool path_found = false;

  while (!open_set.empty()) {
    AStarNode top = open_set.top();
    open_set.pop();
    CellIndex current = top.index;

    if (current == goal) {
      path_found = true;
      break;
    }

    // Skip stale nodes in the open set
    if (g_score.count(current) && top.f_score > g_score[current] + heuristic(current, goal) + 1e-5) {
      continue;
    }

    for (int i = 0; i < 8; ++i) {
      CellIndex neighbor(current.x + dx[i], current.y + dy[i]);

      if (neighbor.x < 0 || neighbor.x >= static_cast<int>(current_map_->info.width) ||
          neighbor.y < 0 || neighbor.y >= static_cast<int>(current_map_->info.height)) {
        continue;
      }

      int index = neighbor.y * current_map_->info.width + neighbor.x;
      int8_t cost = current_map_->data[index];

      if (cost < 0 || cost > 50) {
        continue;
      }

      double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.414 : 1.0;
      double tentative_g = g_score[current] + step_cost;

      if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.push({neighbor, f});
      }
    }
  }

  if (path_found) {
    std::vector<CellIndex> grid_path;
    CellIndex curr = goal;
    while (curr != start) {
      grid_path.push_back(curr);
      curr = came_from[curr];
    }
    grid_path.push_back(start);
    std::reverse(grid_path.begin(), grid_path.end());

    for (const auto &cell : grid_path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = current_map_->header.frame_id;
      pose.header.stamp = path.header.stamp;
      gridToWorld(cell, *current_map_, pose.pose.position.x, pose.pose.position.y);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
  }

  return path;
}

}