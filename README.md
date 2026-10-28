# rrt_planner — ROS 2

Framework to implement RRT global planners as a plugin for **Nav2** (Navigation2), the ROS 2 navigation stack.

> **Mini-Project 2 — Task 1b**  
> Clone this repository, implement the skeleton functions, and run the RRT planner as a Nav2 global planner plugin. Refer to the Nav2 plugin tutorial at https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html

## Demo

![RRT planner demo](media/rrt_turtlebot_rviz.gif)

## Plugin interface

This package implements `nav2_core::GlobalPlanner`. The planner server loads it at runtime via `pluginlib`. The entry point is `rrt_planner::RRTPlannerROS`, which inherits the Nav2 lifecycle interface:

| Method | Purpose |
|---|---|
| `configure()` | Called on plugin load — reads parameters, creates publisher, instantiates the planner |
| `activate()` | Called on server activation — activates the publisher |
| `deactivate()` | Called on server deactivation |
| `cleanup()` | Called on unload — releases resources |
| `createPlan(start, goal)` | Called each time Nav2 needs a global path — returns `nav_msgs::msg::Path` |

## Files to implement

Students must fill in the `/** Implement your code here **/` sections in:

- `src/rrt_planner.cpp` — `getNearestNodeId()`, `createNewNode()`, `sampleRandomPoint()`, `extendTree()`
- `src/collision_detector.cpp` — `inFreeSpace()`

The Nav2 interface (`rrt_planner_ros.cpp`) and the ROS 2 build system (`CMakeLists.txt`, `package.xml`) are already complete.

## Install dependencies and build

```bash
cd ~/ros2_ws/src
git clone https://github.com/irob-labs-ist/rrt_planner.git -b ros2

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rrt_planner
source install/setup.bash
```

## Usage

### 1. Run the full navigation stack with the RRT planner

Provide the map produced in Mini-Project 1.

**TurtleBot 3:**
```bash
ros2 launch rrt_planner rrt_turtlebot.launch.py \
    map:=/path/to/map.yaml \
    use_sim_time:=true        # set false for a real robot
```

**TurtleBot 4:**
```bash
ros2 launch rrt_planner rrt_turtlebot.launch.py \
    map:=/path/to/map.yaml \
    turtlebot:=4 \
    use_sim_time:=true
```

This starts Nav2 (`planner_server`, `controller_server`, `amcl`, `map_server`, …) with `rrt_planner/RRTPlannerROS` set as the global planner.

### 2. Set a goal in RViz2

Use the **2D Goal Pose** tool in RViz2 to send a `NavigateToPose` goal. The RRT planner will be called and the resulting path published on the `/plan` topic.

### 3. Set a goal programmatically (Task 2a)

Use Nav2's `NavigateToPose` action client:

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped

navigator = BasicNavigator()
goal = PoseStamped()
goal.header.frame_id = 'map'
goal.pose.position.x = 1.0
goal.pose.position.y = 0.5
navigator.goToPose(goal)
```

See https://docs.nav2.org/tutorials/docs/navigate_to_pose.html for a complete example.

## Parameters

Parameters are set in `config/rrt_planner_nav2_params.yaml` and can be tuned to explore different RRT behaviours (see Mini-Project 2, Task 1b):

| Parameter | Default | Description |
|---|---|---|
| `goal_tolerance` | 0.20 m | Distance to goal that counts as success |
| `rrt.step` | 0.20 m | Tree extension step size |
| `rrt.min_num_nodes` | 200 | Minimum iterations before checking goal tolerance |
| `rrt.max_num_nodes` | 5000 | Maximum iterations before giving up |

## Collision detection

Collision checking uses `nav2_costmap_2d` (the ROS 2 replacement for `costmap_2d`). The `CollisionDetector` class queries the global costmap to detect obstacles during tree extension. Refer to https://docs.nav2.org/configuration/packages/configuring-costmaps.html for costmap configuration.

## Published topics

| Topic | Type | Description |
|---|---|---|
| `/plan` | `nav_msgs/msg/Path` | The global path computed by the RRT planner (for visualisation in RViz2 / Foxglove) |
