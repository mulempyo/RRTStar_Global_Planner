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
#include <visualization_msgs/MarkerArray.h>

PLUGINLIB_EXPORT_CLASS(rrt_star::RRTStarPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_star {

RRTStarPlanner::RRTStarPlanner() : initialized_(false), goal_threshold_(0.5), step_size_(0.05), max_iterations_(100000), rewire_radius_(0.05) {}

RRTStarPlanner::RRTStarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false), goal_threshold_(0.5), step_size_(0.05), max_iterations_(100000), rewire_radius_(0.05) {
    initialize(name, costmap_ros);
}

RRTStarPlanner::~RRTStarPlanner() {
    if (world_model_) {
        delete world_model_;
    }
}

void RRTStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) {

  if (!initialized_) {
    ROS_WARN("rrt* initialize");
    ros::NodeHandle private_nh("~/" + name);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("rrt_star_plan",1);
    tree_pub_ = private_nh.advertise<visualization_msgs::Marker>("rrt_star_tree", 1);
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
    ROS_WARN("RRTstarPlanner has already been initialized.");
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
            unsigned int new_x_int, new_y_int, new_index, cost;

            if(isValidPose(new_x,new_y)){
             costmap_->worldToMap(new_x, new_y, new_x_int, new_y_int);
             cost = costmap_->getCost(new_x_int, new_y_int);
             new_index = new_y_int * width_ + new_x_int;
            }

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
     visualizeTree();
    }

    return constructPath(start_index, goal_index, plan);
}

void RRTStarPlanner::rewire(unsigned int new_index) {
    double new_x, new_y;
    costmap_->mapToWorld(new_index % width_, new_index / width_, new_x, new_y);

    unsigned int closest_neighbor = new_index;
    double min_cost = std::numeric_limits<double>::max();

    for (const auto& node : tree_) {
        unsigned int neighbor_index = node.first;
        if (neighbor_index == new_index) continue;

        double neighbor_x, neighbor_y;
        costmap_->mapToWorld(neighbor_index % width_, neighbor_index / width_, neighbor_x, neighbor_y);

        if (distance(new_x, new_y, neighbor_x, neighbor_y) <= rewire_radius_ && isValidPose(new_x, new_y) && isValidPose(neighbor_x, neighbor_y)
            && isWithinMapBounds(new_x, new_y) && isWithinMapBounds(neighbor_x, neighbor_y)) {
            double potential_cost = costs_[new_index] + distance(new_x, new_y, neighbor_x, neighbor_y);

            double obstacle_cost = footprintCost(neighbor_x, neighbor_y, 0.0);
            potential_cost += obstacle_cost * 10.0; 

            if (potential_cost < min_cost && isValidPathBetweenPoses(new_x, new_y, 0.0, neighbor_x, neighbor_y, 0.0)) {
                closest_neighbor = neighbor_index;
                min_cost = potential_cost;
            }
        }
    }

    if (closest_neighbor != new_index) {
        costs_[closest_neighbor] = min_cost;
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
    ROS_ERROR("The RRTstarPlanner has not been initialized, you must call initialize().");
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

bool RRTStarPlanner::isValidPose(double x, double y) const {
    
    double obstacle_radius = 0.3;  
    unsigned int mx, my, cost;
    if (costmap_->worldToMap(x, y, mx, my)) {
        for (int dx = -3; dx <= 3; ++dx) {
            for (int dy = -3; dy <= 3; ++dy) {
                unsigned int nx = mx + dx;
                unsigned int ny = my + dy;
                if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
                cost = costmap_->getCost(nx, ny);
                if (cost > 128) {
                    return false;  
                }
            }
        }
    }

    if(cost == costmap_2d::FREE_SPACE){
      return true;
    }
}

void RRTStarPlanner::createRandomValidPose(double &x, double &y, double &th) const {
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

    int max_attempts = 100;  
    int attempts = 0;

    while (!found_pose && attempts < max_attempts) {
        double wx_rand = wx_min + dis(gen) * (wx_max - wx_min);
        double wy_rand = wy_min + dis(gen) * (wy_max - wy_min);
        double th_rand = -M_PI + dis(gen) * (2.0 * M_PI);

        if (isValidPose(wx_rand, wy_rand, th_rand) && isValidPose(wx_rand, wy_rand)) {
            x = wx_rand;
            y = wy_rand;
            th = th_rand;
            found_pose = true;
        }

        attempts++;
    }

    if (!found_pose) {
        ROS_WARN("Failed to find a valid pose after %d attempts. Returning last sample.", max_attempts);
    }
}

unsigned int RRTStarPlanner::nearestNode(double random_x, double random_y) {
    unsigned int nearest_index = 0;
    double min_cost = std::numeric_limits<double>::max();
    double min_dist = std::numeric_limits<double>::max();

    for (const auto& node : tree_) {
        unsigned int node_index = node.first;
        double node_x, node_y;
        costmap_->mapToWorld(node_index % width_, node_index / width_, node_x, node_y);

        double dist = distance(node_x, node_y, random_x, random_y);

        // Check if the node is within rewire_radius_
        if (dist <= rewire_radius_ && dist > 0.001 && isValidPose(node_x, node_y) && isValidPose(random_x, random_y)
            && isWithinMapBounds(node_x, node_y) && isWithinMapBounds(random_x, random_y)) {
            double node_cost = costs_[node_index]; // Retrieve the cost of the current node

            // Choose the node with the smallest cost within the radius
            if (node_cost < min_cost || (node_cost == min_cost && dist < min_dist)){
            if(isValidPose(node_x, node_y) && isValidPose(random_x, random_y) 
               && isWithinMapBounds(node_x, node_y) && isWithinMapBounds(random_x, random_y)){
                min_cost = node_cost;
                min_dist = dist;
                nearest_index = node_index;
              }else{
                continue;
              }
            }
        }
    }

    // If no node is found within the radius, fallback to the globally nearest node
    if (nearest_index == 0) {
        for (const auto& node : tree_) {
            unsigned int node_index = node.first;
            double node_x, node_y;
            costmap_->mapToWorld(node_index % width_, node_index / width_, node_x, node_y);

            double dist = distance(node_x, node_y, random_x, random_y);

            if (dist < min_dist && dist > 0.001){
                if(isValidPose(node_x, node_y) && isWithinMapBounds(node_x, node_y)) {
                 min_dist = dist;
                 nearest_index = node_index;
                }else{
                    continue;
                }
            }
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

  double newX, newY, newTh;

  if (mag < 0.001) {
    new_x = end_x;
    new_y = end_y;
    new_th = end_th;
    return;
  }

  x_step /= mag;
  y_step /= mag;

  newX = start_x + x_step * range;
  newY = start_y + y_step * range;
  newTh = start_th;

  if(isValidPose(newX, newY)){
    new_x = newX;
    new_y = newY;
    new_th = newTh;
  }
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

        if (!isValidPose(interp_x, interp_y, interp_th)) {
            return false; 
        }

        
        current_step += interp_step_size;
        
    }

    return true;
}

bool RRTStarPlanner::isWithinMapBounds(double x, double y) const {
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) {
        return false;
    }
    return true;
}

void RRTStarPlanner::visualizeTree() const {
    if (!initialized_) {
        ROS_WARN("RRTstarPlanner has not been initialized.");
        return;
    }

    visualization_msgs::Marker tree_marker;
    tree_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
    tree_marker.header.stamp = ros::Time::now();
    tree_marker.ns = "rrt_star_tree";
    tree_marker.id = 0;
    tree_marker.type = visualization_msgs::Marker::LINE_LIST;
    tree_marker.action = visualization_msgs::Marker::ADD;
    tree_marker.scale.x = 0.02;  // Line thickness
    tree_marker.color.r = 0.0;
    tree_marker.color.g = 0.8;
    tree_marker.color.b = 0.2;
    tree_marker.color.a = 1.0;   // Fully opaque

    tree_marker.pose.orientation.x = 0.0;
    tree_marker.pose.orientation.y = 0.0;
    tree_marker.pose.orientation.z = 0.0;
    tree_marker.pose.orientation.w = 1.0;
    // Add lines for each edge in the tree
    for (const auto& edge : tree_) {
        unsigned int parent_index = edge.second;
        unsigned int child_index = edge.first;

        double parent_x, parent_y, child_x, child_y;
        costmap_->mapToWorld(parent_index % width_, parent_index / width_, parent_x, parent_y);
        costmap_->mapToWorld(child_index % width_, child_index / width_, child_x, child_y);

        // Only add nodes that are within map bounds
        if (isWithinMapBounds(parent_x, parent_y) && isWithinMapBounds(child_x, child_y)) {
            geometry_msgs::Point parent_point, child_point;
            parent_point.x = parent_x;
            parent_point.y = parent_y;
            parent_point.z = 0.0;

            child_point.x = child_x;
            child_point.y = child_y;
            child_point.z = 0.0;

            tree_marker.points.push_back(parent_point);
            tree_marker.points.push_back(child_point);
        }
    }

    // Publish the tree visualization
    tree_pub_.publish(tree_marker);
}



void RRTStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path) const {
  if (!initialized_) {
    ROS_ERROR(
        "The RRTstarPlanner has not been initialized, you must call "
        "initialize().");
    return;
  }

  nav_msgs::Path path_visual;
  path_visual.poses.resize(path.size());
  //ROS_INFO("path size: %f",path.size());

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
