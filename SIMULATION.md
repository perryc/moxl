# Moxl Simulation Guide

## Quick Start (Native Ubuntu with ROS2 Jazzy)

```bash
git clone git@github.com:perryc/moxl.git
cd moxl

# Install ROS2 dependencies
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization ros-jazzy-nmea-navsat-driver \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-diff-drive-controller ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-twist-mux \
  ros-jazzy-foxglove-bridge ros-jazzy-ros-gz \
  python3-pyproj python3-shapely python3-numpy

rosdep install --from-paths . src/toolpath_planner --ignore-src -r -y

# Build
source /opt/ros/jazzy/setup.bash
colcon build --base-paths . src/toolpath_planner

# Launch simulation
source install/setup.bash
ros2 launch moxl sim.launch.py
```

After editing Python files, rebuild just the toolpath_planner:
```bash
colcon build --packages-select toolpath_planner
source install/setup.bash
```

## Quick Start (Docker — slower iteration)

```bash
docker compose build sim
docker compose up sim -d
docker compose logs sim -f          # watch startup
# Wait for "Managed nodes are active"

# Send a mowing mission
docker compose exec sim bash -c \
  "source /opt/ws/install/setup.bash && \
   ros2 action send_goal /moxl/mission moxl/action/MowMission \
   '{airstrip_id: CDS2, runway_id: \"11/29\"}' --feedback"

# Teardown (MUST use -v to clear build cache volumes)
docker compose down -v
```

Docker gotchas:
- Always `docker compose down -v` (not just `down`) — named volumes cache old builds
- `init: true` is set in compose to prevent zombie processes
- Rebuild cycle is ~3 min (build image + Gazebo startup + Nav2 lifecycle)

## Sending a Mission

Once "Managed nodes are active" appears in logs:

```bash
ros2 action send_goal /moxl/mission moxl/action/MowMission \
  '{airstrip_id: CDS2, runway_id: "11/29"}' --feedback
```

## Monitoring

```bash
# Foxglove Studio — connect to ws://localhost:8765
# Useful topics:
#   /moxl/toolpath/all          — full mowing path (lat/lon in pose.position.x/y)
#   /moxl/toolpath/markers      — RViz markers for toolpath visualization
#   /moxl/mission/status        — mission state machine status
#   /moxl/engine/status         — engine state (stub)
#   /moxl/blades/status         — blade state (stub)
#   /gps/fix                    — GPS position
#   /cmd_vel_nav                — velocity commands from Nav2

# Check Nav2 lifecycle
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server

# Check TF tree
ros2 run tf2_tools view_frames
```

## Current State & Bug to Fix

### What works
- Gazebo sim launches, mower spawns at CDS2 airstrip origin
- GPS publishes correct lat/lon, EKF produces odom and map frames
- Nav2 fully activates (all lifecycle nodes active)
- Toolpath generates 18 strips (perimeter + 4 headlands + 13 main strips)
- 1107 waypoints, densified every 5m, with correct heading orientations
- Mission state machine sequences: PREFLIGHT → MOWING → RETURNING → PARKED
- Engine/blade controllers respond (stubs — instant success)

### What's broken: FollowPath "0 poses" error

The mission node sends 1107 poses to Nav2's `FollowPath` action (bypassing the
global planner), but the controller_server immediately rejects with:
```
[controller_server]: Resulting plan has 0 poses in it.
[controller_server]: [follow_path] [ActionServer] Aborting handle.
```

**Root cause hypothesis**: TF timing. The path poses are in `map` frame but the
controller_server needs to transform them to the local costmap frame (`odom`).
If the `map→odom` transform isn't available or is stale when FollowPath starts,
the transform fails silently and the controller sees an empty plan.

**Things to try** (in order of likelihood):

1. **Add a delay before FollowPath** — wait for TF to be ready:
   ```python
   # In mission_node.py _execute_mission(), before _follow_path():
   # Wait for map->odom TF to be available
   import tf2_ros
   tf_buffer = tf2_ros.Buffer()
   tf2_ros.TransformListener(tf_buffer, self)
   tf_buffer.lookup_transform('odom', 'map', rclpy.time.Time(), timeout=Duration(seconds=10))
   ```

2. **Check the path frame** — controller_server may want poses in `odom` not `map`.
   Try changing `path_msg.header.frame_id = "odom"` and converting poses to odom frame.

3. **Check pose timestamps** — each PoseStamped's header.stamp may need to be `Time(0)`
   (latest available) rather than `now()` for TF lookups to work in sim time.

4. **Test with fewer poses** — send just 5-10 poses to rule out message size.

5. **Check controller_server logs at DEBUG level** — `ros2 param set /controller_server
   use_sim_time True` and increase log level to see why the plan is empty.

### Previous planner bugs (why we bypass the planner)

We tried using `NavigateThroughPoses` (which uses the global planner) but hit:

- **NavfnPlanner**: "Goal Coordinates outside bounds" — rolling window costmap origin
  doesn't update properly, reports goals as outside bounds even when 5m from robot
- **SmacPlanner2D**: "exceeded maximum iterations" — 50K iterations not enough for
  362m path on 500x500 grid, and increasing iterations makes it too slow

The mowing field has NO obstacles, so planning is unnecessary. `FollowPath` sends
the pre-computed toolpath directly to the RegulatedPurePursuitController, which is
the correct architecture for open-field autonomous mowing.

## Nav2 Configuration Lessons

These are hard-won from debugging sessions — don't change without understanding why:

| Setting | Value | Why |
|---------|-------|-----|
| `plugins: ["inflation_layer"]` | inflation_radius: 0.0 | `plugins: []` crashes Nav2. Must have at least one plugin. |
| `use_sim_time: True` | On ALL nodes | Missing on planner/behavior_server causes TF wall-clock vs sim-time mismatch — goals "succeed" instantly without driving |
| `rolling_window: true` | Global costmap | No static map exists. Rolling window follows the robot. |
| `width/height: 500` | 1.0m resolution | 250K cells. Smaller → "goal outside bounds". Larger → planner too slow. |
| `transform_tolerance: 2.0` | Both costmaps | Gazebo sim has TF timing lag. Without this, costmap can't find transforms. |
| `max_velocity: [1.2, 0.0, 1.0]` | velocity_smoother | Default 0.26 m/s is far too slow. Must exceed desired 1.0 m/s mowing speed. |
| `SmacPlanner2D` | planner_server | NavfnPlanner has rolling-window bugs. StraightLine not installed by default. |

## Mowing Pattern

The strip_generator.py implements a headland-first agricultural pattern:

```
1. PERIMETER — mow around the full rectangle edge (inset by half cutting width)
2. HEADLANDS — two cross-cuts at each short end (provides turning room)
3. MAIN STRIPS — parallel end-to-end cuts with boustrophedon (alternating) direction
```

All waypoints are densified every 5m and returned as a single continuous path.
With 1.52m cutting width and 0.05m (2") overlap: ~13 main strips + perimeter + 4 headlands = 18 total.

## File Reference

| File | Purpose |
|------|---------|
| `launch/sim.launch.py` | Gazebo + Nav2 + localization + mission nodes |
| `config/nav2_params.yaml` | Nav2 planner, controller, costmap, smoother config |
| `config/mowing_params.yaml` | Cutting width, overlap, speed, airstrip selection |
| `src/toolpath_planner/toolpath_planner/strip_generator.py` | Headland-first strip generation |
| `src/toolpath_planner/nodes/mission_node.py` | Mission state machine + FollowPath integration |
| `src/toolpath_planner/nodes/toolpath_node.py` | ROS2 node that publishes toolpath |
| `src/toolpath_planner/nodes/engine_controller_node.py` | Engine control stub |
| `src/toolpath_planner/nodes/blade_controller_node.py` | Blade control stub |
| `src/toolpath_planner/nodes/safety_monitor_node.py` | GPS watchdog + e-stop |
| `worlds/empty.sdf` | Gazebo world with CDS2 runway visuals |
| `config/airstrips/CDS2.json` | RTK-surveyed runway corner coordinates |
