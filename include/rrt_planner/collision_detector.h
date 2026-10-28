#ifndef RRT_PLANNER_COLLISION_DETECTOR_H
#define RRT_PLANNER_COLLISION_DETECTOR_H

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "rrt_planner/utils.h"
#include "geometry_msgs/msg/point.hpp"
#include <math.h>
#include <vector>

namespace rrt_planner {

class CollisionDetector {

public:
    /**
     * @brief   Constructor for the CollisionDetector object.
     * @param   costmap  Pointer to the Nav2 costmap ROS wrapper used for planning.
     */
    explicit CollisionDetector(nav2_costmap_2d::Costmap2DROS * costmap);

    /**
     * @brief   Check whether world_pos is in free space.
     * @param   world_pos  2D coordinates (x, y) expressed in the world frame.
     * @return  True if the position is free, false otherwise.
     */
    bool inFreeSpace(const double * world_pos);

    /**
     * @brief   Check whether there is an obstacle between point_a and point_b.
     * @param   point_a  2D coordinates (x, y) expressed in the world frame.
     * @param   point_b  2D coordinates (x, y) expressed in the world frame.
     * @return  True if an obstacle is found between the two points, false otherwise.
     */
    bool obstacleBetween(const double * point_a, const double * point_b);

private:
    nav2_costmap_2d::Costmap2D * costmap_;
    double robot_radius_;
    double resolution_, origin_x_, origin_y_;
};

}  // namespace rrt_planner

#endif  // RRT_PLANNER_COLLISION_DETECTOR_H
