# Robot Navigation Stack — Design

This document describes the ROS 2 packages under `src/robot` and the reasoning
behind the main design choices.

## Architecture

```
        /lidar                 /odom/filtered (chassis pose in sim_world)
          │                          │
   ┌──────▼───────┐          ┌──────▼───────┐
   │  costmap     │          │   planner    │
   │ (local grid) │          │   (A* path)  │
   └──────┬───────┘          └──────┬───────┘
          │ /costmap (map frame)    │ /path
   ┌──────▼───────┐          ┌──────▼───────┐
   │  map_memory  │          │   control    │
   │ (global map) │          │ (follow+LCM) │
   └──────┬───────┘          └──────┬───────┘
          │ /map                    │ /cmd_vel (+ chassis relay)
   ┌──────▼───────┐                 ▼
   │   planner    │           ROS imitator / LCM chassis
   └──────────────┘
```

* `odometry_spoof` — publishes `/odom/filtered` from the `sim_world` →
  `robot/chassis` transform (the lidar sits 0.8 m forward of the chassis;
  using the chassis pose avoids position swing while rotating).
* `costmap` — 2-D local costmap centred on the robot. Publishes in the `map`
  frame (cells are world-aligned), so consumers can overlay it directly.
* `map_memory` — fuses the local costmap into a fixed global grid.
* `planner` — A* on the global map, replanning every few seconds and on new
  goals (`/goal_point` or `/move_base_simple/goal`).
* `control` — follows the path with a look-ahead point; publishes twists on
  `/cmd_vel` and mirrors turn commands onto the LCM-driven chassis.

## Key design decisions

### The costmap grows with the robot
The grid starts 10×10 m and is extended (in ~1 m chunks, preserving old cells)
whenever a scan reaches beyond its edge. The result is a compact map that
follows the robot everywhere — an explicit "unreachable wall" at ±10 m or a
hard-coded 50×50 m grid are both avoided. Config: `costmap_size`,
`resolution`.

### Circular, unbounded inflation
Obstacles are inflated with a chamfer (two-sweep) dilation: cost falls
linearly from 100 at the obstacle to 0 at `inflation_radius`, exactly, with no
square-edges or ring of "100" cells. Because the pass is grid-agnostic, it
still works after the map has grown. Config: `inflation_radius`.

### The global map remembers obstacles
`map_memory` only writes cells whose new observation is occupied (`cost > 0`).
Free cells never erase knowledge, so the global map keeps obstacles that the
robot has already passed. This is what makes the "wall" demo persist in the
whole-map view after the robot moves away.

### Params in `config/params.yaml` only
Nodes declare their parameters in code with defaults and the launch file loads
each package's `config/params.yaml` (e.g. `costmap_node:
ros__parameters:`). Changing grid size, speeds, or map extents requires no
recompile.

### Chassis bridge (LCM relay)
The Husarion chassis is driven over LCM and ignores stale commands. The
`control_node` therefore feeds the turn command to `lcm_relay`, which beats it
onto `/cmd_vel` while a turn is requested, so the chassis always receives
fresh motion guidance during turns.

### One node owns the map frame
Only the costmap sets its grid origin (world-aligned), and map memory overlays
it directly — there is no second coordinate transform to drift. Odometry only
enters at the costmap, which already receives world-frame poses.

## Known limitations

* The costmap treats a scan as ground truth: obstacles that were seen but are
  no longer in view stay marked until the map grows and the region is
  re-scanned.
* `map_memory` intentionally never clears old obstacles (see above); a
  permanently relocated obstacle would leave a ghost in the global map.
* The chassis bridge only relays turns; straight-line driving relies on the
  chassis keeping its last command. If the chassis requires continuous beats,
  extend `LcmRelay::timerCallback` to always publish.

## Build & run

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch bringup_robot robot.launch.py
```
