#include <rrt_star_planner/rrt_star.h>
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <cmath>
#include <random>
#include <limits>
#include <algorithm>
#include <mutex>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(rrt_star::RRTStarPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_star {

RRTStarPlanner::RRTStarPlanner() : initialized_(false), goal_threshold_(0.5), step_size_(0.05), max_iterations_(10000), rewire_radius_(0.5) {}

RRTStarPlanner::RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false), goal_threshold_(0.5), step_size_(0.05), max_iterations_(10000), rewire_radius_(0.5) {
    initialize(name, costmap_ros);
}

RRTStarPlanner::~RRTStarPlanner() {
    if (world_model_) {
        delete world_model_;
    }
}

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {

  if (!initialized_) {
    ros::NodeHandle private_nh("~/" + name);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("rrt_star_plan",1);
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();
    resolution_ = costmap_->getResolution();
    width_ = costmap_->getSizeInCellsX();
    height_ = costmap_->getSizeInCellsY();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
    initialized_ = true;
  } else {
    ROS_WARN("RRTPlanner has already been initialized.");
  }
}

bool RRTStarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);

    if (!initialized_) {
        ROS_ERROR("RRTPlanner has not been initialized, please call initialize() before use.");
        return false;
    }

    plan.clear();
    tree_.clear();
    costs_.clear();

    unsigned int start_x, start_y, goal_x, goal_y;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
        ROS_WARN("The start is out of the map bounds.");
        return false;
    }

    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
        ROS_WARN("The goal is out of the map bounds.");
        return false;
    }

    unsigned int start_index = start_y * width_ + start_x;
    unsigned int goal_index = goal_y * width_ + goal_x;

    tree_.emplace_back(start_index, start_index);
    costs_[start_index] = 0.0;

    for (int i = 0; i < max_iterations_; ++i) {
        double random_x, random_y, random_th;
        createRandomValidPose(random_x, random_y, random_th);

        unsigned int nearest_index = nearestNode(random_x, random_y);
        if (nearest_index == std::numeric_limits<unsigned int>::max()) {
            continue;
        }

        double nearest_x, nearest_y;
        costmap_->mapToWorld(nearest_index % width_, nearest_index / width_, nearest_x, nearest_y);

        double new_x, new_y, new_th;
        createPoseWithinRange(nearest_x, nearest_y, 0.0, random_x, random_y, random_th, step_size_, new_x, new_y, new_th);

        if (isValidPathBetweenPoses(nearest_x, nearest_y, 0.0, new_x, new_y, 0.0)) {
            unsigned int new_x_int, new_y_int;
            costmap_->worldToMap(new_x, new_y, new_x_int, new_y_int);
            unsigned int new_index = new_y_int * width_ + new_x_int;

            if (costs_.find(new_index) == costs_.end()) {
                tree_.emplace_back(new_index, nearest_index);
                costs_[new_index] = costs_[nearest_index] + distance(nearest_x, nearest_y, new_x, new_y);

                rewire(new_index);

                if (isValidPathBetweenPoses(new_x, new_y, 0.0, goal.pose.position.x, goal.pose.position.y, 0.0)) {
                    tree_.emplace_back(goal_index, new_index);
                    costs_[goal_index] = costs_[new_index] + distance(new_x, new_y, goal.pose.position.x, goal.pose.position.y);
                    break;
                }
            }
        }
    }

    return constructPath(start_index, goal_index, plan);
}

void RRTStarPlanner::rewire(unsigned int new_index) {
    double new_x, new_y;
    costmap_->mapToWorld(new_index % width_, new_index / width_, new_x, new_y);

    // Variables to keep track of the closest neighbor
    unsigned int closest_neighbor = new_index;
    double min_cost = std::numeric_limits<double>::max();

    for (const auto& node : tree_) {
        unsigned int neighbor_index = node.first;
        if (neighbor_index == new_index) continue;

        double neighbor_x, neighbor_y;
        costmap_->mapToWorld(neighbor_index % width_, neighbor_index / width_, neighbor_x, neighbor_y);

        // Check if the neighbor is within the rewire radius
        if (distance(new_x, new_y, neighbor_x, neighbor_y) <= rewire_radius_) {
            double potential_cost = costs_[new_index] + distance(new_x, new_y, neighbor_x, neighbor_y);

            // Only consider neighbors where the path is valid
            if (potential_cost < min_cost && isValidPathBetweenPoses(new_x, new_y, 0.0, neighbor_x, neighbor_y, 0.0)) {
                closest_neighbor = neighbor_index;
                min_cost = potential_cost;
            }
        }
    }

    // If a closest neighbor is found, rewire to it
    if (closest_neighbor != new_index) {
        costs_[closest_neighbor] = min_cost;

        // Update the tree structure to set the new parent
        auto it = std::find_if(tree_.begin(), tree_.end(), [closest_neighbor](const std::pair<unsigned int, unsigned int>& node) {
            return node.first == closest_neighbor;
        });
        if (it != tree_.end()) {
            it->second = new_index;
        }
    }
}


bool RRTStarPlanner::constructPath(unsigned int start_index, unsigned int goal_index, std::vector<geometry_msgs::PoseStamped>& plan) {
    if (costs_.find(goal_index) == costs_.end()) {
        ROS_WARN("No valid path found to goal.");
        return false;
    }

    unsigned int current_index = goal_index;
    while (current_index != start_index) {
        double wx, wy;
        costmap_->mapToWorld(current_index % width_, current_index / width_, wx, wy);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1));
        plan.push_back(pose);

        auto it = std::find_if(tree_.begin(), tree_.end(), [current_index](const std::pair<unsigned int, unsigned int>& node) {
            return node.first == current_index;
        });

        if (it == tree_.end()) {
            ROS_WARN("Failed to backtrack path.");
            return false;
        }

        current_index = it->second;
    }

    std::reverse(plan.begin(), plan.end());
    publishPlan(plan);
    return true;
}


double RRTStarPlanner::footprintCost(double x, double y, double th) const {
  if (!initialized_) {
    ROS_ERROR("The RRT Planner has not been initialized, you must call initialize().");
    return -1.0;
  }

  std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

  if (footprint.size() < 3) return -1.0;

  double footprint_cost = world_model_->footprintCost(x, y, th, footprint);

  return footprint_cost;
}

bool RRTStarPlanner::isValidPose(double x, double y, double th) const {
  double footprint_cost = footprintCost(x, y, th);
  if ((footprint_cost < 0) || (footprint_cost > 128)) {
    return false;
  }
  return true;
}

void RRTStarPlanner::createRandomValidPose(double &x, double &y, double &th) const {
  // get bounds of the costmap in world coordinates
  double wx_min, wy_min;
  costmap_->mapToWorld(0, 0, wx_min, wy_min);

  double wx_max, wy_max;
  unsigned int mx_max = costmap_->getSizeInCellsX();
  unsigned int my_max = costmap_->getSizeInCellsY();
  costmap_->mapToWorld(mx_max, my_max, wx_max, wy_max);

  bool found_pose = false;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 1.0);

  while (!found_pose) {
    double wx_rand = dis(gen);
    wx_rand = wx_min + wx_rand * (wx_max - wx_min);

    double wy_rand = dis(gen);
    wy_rand = wy_min + wy_rand * (wy_max - wy_min);

    double th_rand = dis(gen);
    th_rand = -M_PI + th_rand * (M_PI - -M_PI);

    if (isValidPose(wx_rand, wy_rand, th_rand)) {
      x = wx_rand;
      y = wy_rand;
      th = th_rand;
      found_pose = true;
    }
  }
}

unsigned int RRTStarPlanner::nearestNode(double random_x, double random_y) {
  unsigned int nearest_index = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto &node : tree_) {
    unsigned int node_index = node.first;
    double node_x,node_y;
    costmap_->mapToWorld(node_index % width_,node_index / width_,node_x,node_y);
    double dist = distance(node_x, node_y, random_x, random_y);
    ROS_INFO("tree node:%d -> %d", node.first, node.second);
    if (dist < min_dist && dist > 0.001) { 
      min_dist = dist;
      nearest_index = node_index;
    }
  }
  return nearest_index;
}

void RRTStarPlanner::createPoseWithinRange(double start_x, double start_y, double start_th,
                                       double end_x, double end_y, double end_th,
                                       double range, double &new_x, double &new_y, double &new_th) const { //world

  double x_step = end_x - start_x;
  double y_step = end_y - start_y;
  double mag = sqrt((x_step * x_step) + (y_step * y_step));

  if (mag < 0.001) {
    new_x = end_x;
    new_y = end_y;
    new_th = end_th;
    return;
  }

  x_step /= mag;
  y_step /= mag;

  new_x = start_x + x_step * range;
  new_y = start_y + y_step * range;
  new_th = start_th;
}

bool RRTStarPlanner::isValidPathBetweenPoses(double x1, double y1, double th1,
                                         double x2, double y2, double th2) const {
  double interp_step_size = 0.05;
  double current_step = interp_step_size;

  double d = std::hypot(x2 - x1, y2 - y1);

  while (current_step < d) {
    double interp_x, interp_y, interp_th;
    createPoseWithinRange(x1, y1, th1, x2, y2, th2, current_step,
                          interp_x, interp_y, interp_th);

    if (!isValidPose(interp_x, interp_y, interp_th)) return false;

    current_step += interp_step_size;
  }

  return true;
}

void RRTStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path) const {
  if (!initialized_) {
    ROS_ERROR(
        "The RRT Planner has not been initialized, you must call "
        "initialize().");
    return;
  }

  nav_msgs::Path path_visual;
  path_visual.poses.resize(path.size());
  ROS_INFO("path size: %f",path.size());

  if (path.empty()) {
    path_visual.header.frame_id = costmap_ros_->getGlobalFrameID();
    path_visual.header.stamp = ros::Time::now();
  } else {
    path_visual.header.frame_id = path[0].header.frame_id;
    path_visual.header.stamp = path[0].header.stamp;
  }

  for (unsigned int i = 0; i < path.size(); i++) {
    path_visual.poses[i] = path[i];
  }

  plan_pub_.publish(path_visual);
}



double RRTStarPlanner::distance(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

void RRTStarPlanner::mapToWorld(unsigned int mx, unsigned int my, double &wx, double &wy) {
  wx = origin_x_ + mx * resolution_;
  wy = origin_y_ + my * resolution_;
}


};  // namespace rrt_star

    
