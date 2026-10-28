#include "rrt_planner/rrt_planner.h"

namespace rrt_planner {

RRTPlanner::RRTPlanner(nav2_costmap_2d::Costmap2DROS * costmap,
        const rrt_params & params) : params_(params), collision_dect_(costmap) {

    costmap_    = costmap->getCostmap();
    map_width_  = costmap_->getSizeInMetersX();
    map_height_ = costmap_->getSizeInMetersY();

    random_double_x.setRange(-map_width_,  map_width_);
    random_double_y.setRange(-map_height_, map_height_);

    nodes_.reserve(params_.max_num_nodes);
}

bool RRTPlanner::planPath() {

    // Clear everything before planning
    nodes_.clear();

    // Start node (root of the tree)
    createNewNode(start_, -1);

    double * p_rand, * p_new;
    Node nearest_node;

    for (unsigned int k = 1; k <= static_cast<unsigned int>(params_.max_num_nodes); k++) {

        p_rand      = sampleRandomPoint();
        nearest_node = nodes_[getNearestNodeId(p_rand)];
        p_new       = extendTree(nearest_node.pos, p_rand);

        if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
            createNewNode(p_new, nearest_node.node_id);
        } else {
            continue;
        }

        if (k > static_cast<unsigned int>(params_.min_num_nodes)) {
            if (computeDistance(p_new, goal_) <= params_.goal_tolerance) {
                return true;
            }
        }
    }

    return false;
}

int RRTPlanner::getNearestNodeId(const double * point) {

    /**************************
     * Implement your code here
     **************************/

}

void RRTPlanner::createNewNode(const double * pos, int parent_node_id) {

    Node new_node;

    /**************************
     * Implement your code here
     **************************/

    nodes_.emplace_back(new_node);
}

double * RRTPlanner::sampleRandomPoint() {

    /**************************
     * Implement your code here
     **************************/

    rand_point_[0] = // ... ;
    rand_point_[1] = // ... ;

    return rand_point_;
}

double * RRTPlanner::extendTree(const double * point_nearest, const double * point_rand) {

    /**************************
     * Implement your code here
     **************************/

    candidate_point_[0] = // ... ;
    candidate_point_[1] = // ... ;

    return candidate_point_;
}

const std::vector<Node> & RRTPlanner::getTree() {
    return nodes_;
}

void RRTPlanner::setStart(double * start) {
    start_[0] = start[0];
    start_[1] = start[1];
}

void RRTPlanner::setGoal(double * goal) {
    goal_[0] = goal[0];
    goal_[1] = goal[1];
}

}  // namespace rrt_planner
