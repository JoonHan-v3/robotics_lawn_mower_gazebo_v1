# Robotic Lawn Mower — Version 1

A ROS 2 + Gazebo simulation of a differential-drive robotic lawn mower: a chassis on two driven wheels and a rear caster, with a spinning cutting blade mounted under a mower deck. Driving it over the lawn with the blade spinning paints a "mowed" trail on the ground.

![demo](version1.mp4)

## Packages

- **`mower_description`** — the robot model.
  - `urdf/robot_base.xacro` — chassis, wheels, caster, deck, blade, bumper (links, joints, inertials).
  - `urdf/common_properties.xacro` — shared materials and inertia macros.
  - `urdf/robot_base_gazebo.xacro` — Gazebo plugins: differential drive, blade joint control, joint state publishing, caster friction.
  - `urdf/robot.urdf.xacro` — top-level file that combines the above into the full robot description.
  - `launch/display.launch.xml` — view the robot in RViz only (no simulation).
  - `rviz/urdf_config.rviz` — RViz display configuration.

- **`mower_bringup`** — simulation bring-up.
  - `worlds/lawn_field.sdf` — a bounded grass field world, with the default Gazebo GUI layout plus a video-recorder toolbar button.
  - `launch/mower.launch.xml` — launches Gazebo, spawns the robot, starts `robot_state_publisher`, the ROS↔Gazebo bridge, the grass-cutting trail node, and RViz.
  - `config/gazebo_bridge.yaml` — topic bridge between ROS 2 and Gazebo (clock, `/cmd_vel`, `/joint_states`, `/tf`, blade command, and ground-truth pose).
  - `scripts/grass_mower.py` — paints a mowed trail behind the deck while the blade is spinning, using the robot's ground-truth simulated pose (not odometry, which drifts).

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic (`gz-sim8`) and `ros_gz`

## Build

```
colcon build
source install/setup.bash
```

## Running the simulation

**Terminal 1 — launch the robot in Gazebo + RViz:**
```
ros2 launch mower_bringup mower.launch.xml
```

**Terminal 2 — drive it with the keyboard:**
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 3 — spin up the blade** (send `0.0` to stop it):
```
ros2 topic pub --once /blade_cmd_vel std_msgs/msg/Float64 "{data: 30.0}"
```

Drive around with the blade spinning and a lighter-green mowed trail will appear behind the mower deck, matching the robot's actual path.

## Recording a video

The Gazebo window includes a record button in the toolbar (added via `worlds/lawn_field.sdf`) that saves an `.mp4` of the 3D view — click it to start, click again to stop and save.

## Viewing the robot model only (no simulation)

```
ros2 launch mower_description display.launch.xml
```
