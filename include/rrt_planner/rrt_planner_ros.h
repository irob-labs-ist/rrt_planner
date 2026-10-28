#ifndef RRT_PLANNER_RRT_PLANNER_ROS_H
#define RRT_PLANNER_RRT_PLANNER_ROS_H

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rrt_planner/rrt_planner.h"
#include "rrt_planner/utils.h"

namespace rrt_planner {

/**
 * @class RRTPlannerROS
 * @brief Nav2 global planner plugin wrapping the RRT planner.
 *
 * Implements the nav2_core::GlobalPlanner interface so it can be loaded
 * by the Nav2 planner server (nav2_bringup / navigation_launch.py).
 */
class RRTPlannerROS : public nav2_core::GlobalPlanner
{
public:
  RRTPlannerROS() = default;
  ~RRTPlannerROS() override = default;

  /**
   * @brief Called by the planner server on plugin load.
   * @param parent   Weak pointer to the lifecycle node that owns this plugin.
   * @param name     Name given to this planner in the Nav2 params file.
   * @param tf       TF2 buffer (available for frame transforms if needed).
   * @param costmap_ros  Shared pointer to the global costmap ROS wrapper.
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /// Called when the planner server transitions to inactive.
  void cleanup() override;

  /// Called when the planner server transitions to active.
  void activate() override;

  /// Called when the planner server transitions back to inactive.
  void deactivate() override;

  /**
   * @brief Compute a global path from start to goal using RRT.
   * @param start  Start pose (must be in the global costmap frame).
   * @param goal   Goal pose  (must be in the global costmap frame).
   * @return       Planned path; an empty path signals failure.
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::shared_ptr<RRTPlanner> planner_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  bool initialized_{false};

private:
  std::string name_;
  std::string global_frame_;
  rrt_params params_;
  std::vector<Node> rrt_tree_;
};

}  // namespace rrt_planner

#endif  // RRT_PLANNER_RRT_PLANNER_ROS_H
