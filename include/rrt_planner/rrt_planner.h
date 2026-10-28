#ifndef RRT_PLANNER_RRT_PLANNER_H
#define RRT_PLANNER_RRT_PLANNER_H

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rrt_planner/collision_detector.h"
#include "rrt_planner/random_double_generator.h"
#include "rrt_planner/utils.h"
#include <vector>

namespace rrt_planner {

class RRTPlanner {

public:
    /**
     * @brief Constructor for the RRTPlanner object.
     * @param costmap A pointer to the Nav2 costmap ROS wrapper used for planning.
     * @param params  Struct with parameters to build the tree.
     */
    RRTPlanner(nav2_costmap_2d::Costmap2DROS * costmap, const rrt_params & params);

    /**
     * @brief   Plan a path using RRT.
     * @return  True if a path was found, false otherwise.
     */
    bool planPath();

    /**
     * @brief   Sample a random point in the map.
     * @return  2D random planar position (x, y).
     */
    double * sampleRandomPoint();

    /**
     * @brief   Get the index of the nearest node to the given random point.
     * @param   point  Random point sampled.
     * @return  Index of the nearest node in nodes_.
     */
    int getNearestNodeId(const double * point);

    /**
     * @brief   Create a new node and add it to the tree.
     * @param   pos           2D coordinates of the new node.
     * @param   parent_node_id  Id of the parent node.
     */
    void createNewNode(const double * pos, int parent_node_id);

    /**
     * @brief   Extend the tree from the nearest node toward the random point.
     * @param   point_nearest  2D coordinates of the nearest existing tree node.
     * @param   point_rand     2D coordinates of the sampled random point.
     * @return  2D coordinates of the potential new tree node.
     */
    double * extendTree(const double * point_nearest, const double * point_rand);

    void setStart(double * start);
    void setGoal(double * goal);

    const std::vector<Node> & getTree();

    ~RRTPlanner() {}

private:
    double start_[2], goal_[2];
    double rand_point_[2], candidate_point_[2];
    float map_width_, map_height_;
    nav2_costmap_2d::Costmap2D * costmap_;
    std::vector<Node> nodes_;
    rrt_params params_;
    CollisionDetector collision_dect_;
    RandomDoubleGenerator random_double_x, random_double_y;
};

}  // namespace rrt_planner

#endif  // RRT_PLANNER_RRT_PLANNER_H
