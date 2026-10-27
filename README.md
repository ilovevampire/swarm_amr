# AMR simulation: Gazebo Harmonic + ROS 2 Jazzy

A differential-drive autonomous mobile robot (AMR) simulated in a warehouse world with
ROS 2 Jazzy and Gazebo Harmonic. The workspace covers the full chain from robot model to
autonomous navigation:

robot description -> Gazebo sensors -> ros2_control -> EKF sensor fusion -> SLAM mapping -> Nav2

> Status: simulation works end to end (spawn, teleop, EKF, SLAM mapping, saved map, Nav2 setup).
> Nav2 tuning and the next phase are still open.

## Workspace layout

```
swarm_ws/
├── src/
│   ├── amr_description/   robot model: URDF/xacro, meshes, controller config, robot RViz view
│   ├── amr_control/       EKF (robot_localization) configuration and launch
│   ├── amr_navigation/    SLAM, localization and Nav2: params, launch files, maps, RViz views
│   └── swarm_gazebo/      Gazebo world, robot spawn launch, bridge to ROS
├── tools/                 small diagnostic / map-repair scripts (not ROS packages)
├── .gitignore
└── README.md
```

| Package | Contents |
|---|---|
| `amr_description` | `urdf/amr.urdf.xacro` (top file) includes `amr_description.urdf.xacro` (links, joints, `ros2_control`) and `amr_gazebo.xacro` (Gazebo sensors, friction, `gz_ros2_control` plugin); `mesh/`; `config/amr_controllers.yaml`; `rviz/amr.rviz` |
| `amr_control` | `config/ekf.yaml`, `launch/ekf.launch.py` |
| `amr_navigation` | `config/` `slam_params.yaml`, `slam_localization.yaml`, `nav2_params.yaml`; `launch/` `slam`, `localization`, `navigation`, `rviz`; `maps/`; `rviz/` `slam.rviz`, `nav.rviz`; `scripts/cmd_vel_stamper.py` |
| `swarm_gazebo` | `worlds/warehouse_world.sdf`; `launch/` `warehouse`, `spawn_amr`, `rviz` |

## The robot

- Differential drive: two drive wheels (radius 0.075 m, track 0.384 m) and four passive caster wheels
  with fixed (non-swivelling) mounts; about 1.5 kg.
- Sensors (all Gazebo, bridged to ROS by `ros_gz_bridge` in `spawn_amr.launch.py`):

| Sensor | Topic | Notes |
|---|---|---|
| 2D lidar (`gpu_lidar`) | `/scan` | 360 rays, 0.12 to 8 m, 30 Hz, frame `lidar_link` |
| Camera | `/camera/image_raw`, `/camera/camera_info` | 640x480, frame `camera_optical_frame` |
| IMU | `/imu/data` | 30 Hz, simulated Gaussian noise |
| 3 ultrasonic | `/ultrasonic/{center,left,right}/scan` | narrow 15 degree beam, 0.05 to 2 m, 10 Hz, left/right angled 15 degrees outward |

- Control: `ros2_control` with `gz_ros2_control`; `diff_drive_controller` (50 Hz) and
  `joint_state_broadcaster`. The controller takes **stamped** velocity commands on
  `/diff_drive_controller/cmd_vel` (`geometry_msgs/TwistStamped`).

### TF tree

```
map ──(slam_toolbox or AMCL)──> odom ──(EKF)──> base_footprint ──> base_link ──> sensor links
```

Exactly one node may publish `odom -> base_footprint`. That node is the EKF, so the controller runs
with `enable_odom_tf: false`.

## Requirements

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic.

```bash
sudo apt install -y \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-robot-localization ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-map-server \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2
```

## Build

```bash
cd ~/swarm_ws
colcon build --symlink-install
source install/setup.bash
chmod +x src/amr_navigation/scripts/cmd_vel_stamper.py   # once, then rebuild
```

Rebuild after adding new files to `maps/`, `rviz/` or `config/`, so the installed copies exist.

## Run

Before every run, in every terminal: `source ~/swarm_ws/install/setup.bash`. Clean up leftovers first:

```bash
pkill -9 -f "gz sim"; pkill -9 ruby
```

### Common start (all modes), one terminal each

```bash
ros2 launch swarm_gazebo warehouse.launch.py        # 1 world (wait until loaded)
ros2 launch swarm_gazebo spawn_amr.launch.py        # 2 robot, bridge, controllers
ros2 launch amr_control ekf.launch.py               # 3 EKF  ("Waiting for clock" line is normal)
```

Quick checks: `ros2 control list_controllers` (both `active`), `ros2 topic hz /clock` (about 200 Hz),
`ros2 topic hz /odometry/filtered` (about 50 Hz).

### A. Build a map (SLAM)

```bash
ros2 launch amr_navigation slam.launch.py                              # 4
ros2 launch amr_navigation rviz.launch.py                              # 5 (slam.rviz, fixed frame map)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p speed:=0.2 -p turn:=0.5 \
  -r cmd_vel:=/diff_drive_controller/cmd_vel                           # 6 teleop
```

Drive slowly (about 0.15 to 0.2 m/s), overlap passes, come back to the start to close loops.

Save the map (while SLAM is running):

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/vshal/swarm_ws/src/amr_navigation/maps/warehouse_v2'}}"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/vshal/swarm_ws/src/amr_navigation/maps/warehouse_v2_posegraph'}"
```

This writes `warehouse_v2.pgm` + `.yaml` (for Nav2) and `warehouse_v2_posegraph.*` (for slam_toolbox).

### B. Localize on a saved map (slam_toolbox)

```bash
ros2 launch amr_navigation localization.launch.py   # instead of slam.launch.py
ros2 launch amr_navigation rviz.launch.py
```

### C. Navigate (Nav2)

```bash
ros2 launch amr_navigation navigation.launch.py                              # 4
ros2 launch amr_navigation rviz.launch.py rviz_config:=nav.rviz              # 5
```

Use **2D Pose Estimate** if the robot is not at the start pose, then **Nav2 Goal** to send a goal.
Do not run SLAM or localization together with Nav2, and do not use teleop while Nav2 drives.

Velocity chain: `controller_server -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed ->
collision_monitor -> /cmd_vel (Twist) -> cmd_vel_stamper -> /diff_drive_controller/cmd_vel`.

## Tools (`tools/`)

| Script | Purpose |
|---|---|
| `scan_check.py` | lidar health: hits, closest hit, no-hit / too-close counts |
| `ultra_check.py` | live closest range of the three ultrasonic sensors |
| `stability_check.py` | robot standing still: wheel jitter, EKF drift, SLAM `map -> odom` jumps |
| `yaw_compare.py` | heading from wheel odometry vs EKF vs SLAM |
| `fix_map.py` | repaint polygons in a saved map (`occupied` / `free` / `unknown`) without touching the original |

## Machine-specific paths

These use the home directory `/home/vshal`; change them on another machine:
- `src/amr_navigation/launch/navigation.launch.py` (`MAP_YAML`)
- `src/amr_navigation/config/slam_localization.yaml` (`map_file_name`)

## Lessons learned

- Gazebo Harmonic needs `gz::sim::...` plugin names, `ogre2`, and `type="gpu_lidar"` with a `<lidar>` block.
  Old `ignition::...` names, `ogre`, or `type="lidar"` with `<ray>` give no data or crashes.
- `package://` mesh paths need `GZ_SIM_RESOURCE_PATH` (set in `warehouse.launch.py`), and the mesh folder name
  must match the URDF and the `install(DIRECTORY ...)` line.
- In a Jazzy launch file, wrap xacro output in `ParameterValue(..., value_type=str)`.
- Two nodes publishing `odom -> base_footprint` corrupt SLAM. Check `ros2 param get /diff_drive_controller enable_odom_tf` is `False`.
- Always kill leftover Gazebo processes before relaunching, or old robots show up in the new session.
- Jazzy `diff_drive_controller` only accepts `TwistStamped`; Nav2 (Jazzy) outputs `Twist`, hence `cmd_vel_stamper`.
- Six ground contacts (2 wheels + 4 casters) make Gazebo wheels chatter; keep wheel torque low and collisions simple.
- Pose graphs are large and are excluded from git (see `.gitignore`); regenerate them by mapping.

## Roadmap

- [x] Robot model, sensors, controllers
- [x] EKF sensor fusion
- [x] SLAM mapping and saved map
- [x] Nav2 configuration
- [ ] Nav2 tuning and testing
- [ ] Next phase: to be defined

## License

Packages declare `Apache-2.0`; set the maintainer name and email in each `package.xml`.
