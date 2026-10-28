#include "rrt_planner/rrt_planner_ros.h"
#include "pluginlib/class_list_macros.hpp"

// Register this planner as a nav2_core::GlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlannerROS, nav2_core::GlobalPlanner)

namespace rrt_planner {

void RRTPlannerROS::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = node_.lock();

  // Declare parameters (name-prefixed so multiple planner instances don't clash)
  node->declare_parameter(name_ + ".goal_tolerance", 0.20);
  node->declare_parameter(name_ + ".rrt.step", 0.20);
  node->declare_parameter(name_ + ".rrt.min_num_nodes", 200);
  node->declare_parameter(name_ + ".rrt.max_num_nodes", 5000);

  params_.goal_tolerance = node->get_parameter(name_ + ".goal_tolerance").as_double();
  params_.step           = node->get_parameter(name_ + ".rrt.step").as_double();
  params_.min_num_nodes  = node->get_parameter(name_ + ".rrt.min_num_nodes").as_int();
  params_.max_num_nodes  = node->get_parameter(name_ + ".rrt.max_num_nodes").as_int();

  // Publisher for the planned path (visualised in RViz2 / Foxglove)
  plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("plan", 1);

  planner_ = std::make_shared<RRTPlanner>(costmap_ros.get(), params_);
  initialized_ = true;

  RCLCPP_INFO(node->get_logger(),
    "[RRTPlannerROS] Configured with goal_tolerance=%.2f, step=%.2f, "
    "min_nodes=%d, max_nodes=%d",
    params_.goal_tolerance, params_.step,
    params_.min_num_nodes, params_.max_num_nodes);
}

void RRTPlannerROS::cleanup()
{
  plan_pub_.reset();
  planner_.reset();
  initialized_ = false;
}

void RRTPlannerROS::activate()
{
  plan_pub_->on_activate();
}

void RRTPlannerROS::deactivate()
{
  plan_pub_->on_deactivate();
}

nav_msgs::msg::Path RRTPlannerROS::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;

  auto node = node_.lock();

  if (!initialized_) {
    RCLCPP_ERROR(node->get_logger(),
      "[RRTPlannerROS] Planner not initialized — call configure() first.");
    return path;
  }

  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node->get_logger(),
      "[RRTPlannerROS] Start pose is in frame '%s', expected '%s'.",
      start.header.frame_id.c_str(), global_frame_.c_str());
    return path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node->get_logger(),
      "[RRTPlannerROS] Goal pose is in frame '%s', expected '%s'.",
      goal.header.frame_id.c_str(), global_frame_.c_str());
    return path;
  }

  double world_start[2]{start.pose.position.x, start.pose.position.y};
  double world_goal[2]{goal.pose.position.x,  goal.pose.position.y};

  unsigned int map_x, map_y;
  if (!costmap_->worldToMap(world_start[0], world_start[1], map_x, map_y)) {
    RCLCPP_WARN(node->get_logger(),
      "[RRTPlannerROS] Start position (%.2f, %.2f) is off the costmap.",
      world_start[0], world_start[1]);
    return path;
  }
  if (!costmap_->worldToMap(world_goal[0], world_goal[1], map_x, map_y)) {
    RCLCPP_WARN(node->get_logger(),
      "[RRTPlannerROS] Goal position (%.2f, %.2f) is off the costmap.",
      world_goal[0], world_goal[1]);
    return path;
  }

  RCLCPP_INFO(node->get_logger(),
    "[RRTPlannerROS] Planning from (%.2f, %.2f) to (%.2f, %.2f)",
    world_start[0], world_start[1], world_goal[0], world_goal[1]);

  planner_->setStart(world_start);
  planner_->setGoal(world_goal);

  if (!planner_->planPath()) {
    RCLCPP_WARN(node->get_logger(), "[RRTPlannerROS] Failed to find a path.");
    return path;
  }

  // Build the path message by walking back from the last node to the root
  rrt_tree_ = planner_->getTree();
  path.header.stamp = node->now();

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = path.header;
  pose_stamped.pose.orientation = goal.pose.orientation;

  int current_id = static_cast<int>(rrt_tree_.size()) - 1;
  int prev_id;
  tf2::Quaternion quat_tf;
  double dy, dx, yaw;

  while (current_id != 0) {
    pose_stamped.pose.position.x = rrt_tree_[current_id].pos[0];
    pose_stamped.pose.position.y = rrt_tree_[current_id].pos[1];
    pose_stamped.pose.position.z = 0.0;
    path.poses.push_back(pose_stamped);

    prev_id    = current_id;
    current_id = rrt_tree_[current_id].parent_id;

    // Compute heading from current node toward its parent
    dy  = rrt_tree_[prev_id].pos[1] - rrt_tree_[current_id].pos[1];
    dx  = rrt_tree_[prev_id].pos[0] - rrt_tree_[current_id].pos[0];
    yaw = std::atan2(dy, dx);
    quat_tf.setRPY(0.0, 0.0, yaw);
    pose_stamped.pose.orientation = tf2::toMsg(quat_tf);
  }

  // Add start pose
  pose_stamped.pose.position.x = rrt_tree_[0].pos[0];
  pose_stamped.pose.position.y = rrt_tree_[0].pos[1];
  pose_stamped.pose.position.z = 0.0;
  pose_stamped.pose.orientation = start.pose.orientation;
  path.poses.push_back(pose_stamped);

  // Reverse so the path runs start → goal
  std::reverse(path.poses.begin(), path.poses.end());

  plan_pub_->publish(path);
  return path;
}

}  // namespace rrt_planner
