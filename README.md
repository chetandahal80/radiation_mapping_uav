# Autonomous Radiation Mapping UAV (PX4 SITL)

**ROS1 Noetic · Ubuntu 20.04 · Gazebo 11**

Single-UAV simulation stack for radiation mapping of radioactive environments.
PX4-Autopilot SITL (Gazebo) flies a simulated iris quadcopter carrying a radiation
sensor; Hector SLAM builds a 2D occupancy grid from onboard LiDAR; the Hector
radiation mapping tool fuses the two into a probabilistic radiation intensity map.
The drone explores autonomously using frontier-based exploration for global
coverage, with `move_base` (navfn global planner + DWA local planner) handling
obstacle-aware navigation to each frontier goal.

## Architecture

PX4 SITL + Gazebo → radiation sensor/source plugins → `radiation_bridge` →
Hector SLAM + `hector_radiation_mapping` → `offboard_py` / `quadcoptor_navigation`
→ MAVROS → PX4.

## Packages

| Package | Tier | Origin | What changed |
|---|---|---|---|
| command_bridge | Own | — | — |
| offboard_py | Own | — | Frontier exploration + OFFBOARD/navigation node |
| radiation_bridge | Own | — | Translates sensor plugin messages into hector_radiation_mapping's expected format |
| radiation_sim_launcher | Own | — | Top-level launch files + RViz config |
| quadcoptor_navigation | move_base package| — | move_base / navfn / DWA integration |
| gazebo_radiation_plugin | Fork | [EEEManchester/gazebo_radiation_plugin](https://github.com/EEEManchester/gazebo_radiation_plugin) | Sensor/source configs, sensor plugin C++ edits |
| hector_radiation_mapping | Fork | [tu-darmstadt-ros-pkg/hector_radiation_mapping](https://github.com/tu-darmstadt-ros-pkg/hector_radiation_mapping) | sampleManager.cpp, gpython.cpp, Sample.msg, launch/params |
| hector_slam | Fork | [tu-darmstadt-ros-pkg/hector_slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam) | Launch params + static tf broadcasters for the iris drone |
| hector_localization (message_to_tf) | Fork | [tu-darmstadt-ros-pkg/hector_localization](https://github.com/tu-darmstadt-ros-pkg/hector_localization) | One-line `child_frame_id` fix in `sendTransform()` |
| PX4-Autopilot | Fork | [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) | New SITL launch scripts; adapted sitl_run/sitl_multiple_run |
| ↳ PX4-SITL_gazebo-classic (submodule) | Fork | [PX4/PX4-SITL_gazebo-classic](https://github.com/PX4/PX4-SITL_gazebo-classic) | Radiation sensor added to iris model; radiation-source worlds (reactor room, empty_radiation) |
| hector_rviz_plugins | Dependency | [tu-darmstadt-ros-pkg/hector_rviz_plugins](https://github.com/tu-darmstadt-ros-pkg/hector_rviz_plugins) | Unmodified |
| mavros | Dependency | [mavlink/mavros](https://github.com/mavlink/mavros) | Unmodified — `fcu_url` overridden at launch time instead of editing the package |
| mavlink | Dependency | [mavlink/mavlink-gbp-release](https://github.com/mavlink/mavlink-gbp-release) | Unmodified |
| grid_map | Dependency | [ANYbotics/grid_map](https://github.com/ANYbotics/grid_map) | Unmodified |
| ddynamic_reconfigure | Dependency | [pal-robotics/ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure) | Unmodified |
| ros_babel_fish | Dependency | [StefanFabian/ros_babel_fish](https://github.com/StefanFabian/ros_babel_fish) | Unmodified |

## Setup

```bash
pip install vcstool
git clone https://github.com/chetandahal80/radiation_mapping_uav.git
cd radiation_mapping_uav
vcs import --recursive src < workspace.repos
rosdep install --from-paths src --ignore-src -r -y
sudo apt install python3-catkin-tools python3-osrf-pycommon
catkin build
```

## Flight controller parameters (set once, before first flight)

Default PX4 flight speeds are too fast for Hector SLAM to keep up with — the
generated map becomes unstable at higher velocity/acceleration. Using
QGroundControl (Vehicle Setup → Parameters), set:

| Parameter | Value | Meaning |
|---|---|---|
| `MPC_XY_VEL_MAX` | 1.0 m/s | Max. horizontal velocity |
| `MPC_XY_VEL_P_ACC` | 1.20 | Proportional gain for horizontal velocity error |
| `MC_YAWRATE_MAX` | 10.0 deg/s | Max. yaw rate |
| `MPC_ACC_HOR_MAX` | 2.50 m/s² | Max. horizontal acceleration |

## Running the Simulation

Three terminals, in this order. Source the workspace (`source devel/setup.bash`) in each one first.

**Terminal 1 — start PX4 SITL + Gazebo with the radiation world:**
```bash
(~/path to radiation_mapping_uav)/src/PX4-Autopilot/run_px4_with_radiation.sh
```

**Terminal 2 — bring up SLAM, radiation mapping, radiation_bridge, and RViz:**
```bash
roslaunch radiation_sim_launcher radiation_mapping_with_sitl.launch
```

**Terminal 3 — connect MAVROS to the simulated flight controller:**
```bash
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

Once all three are running, the drone begins autonomous frontier-based exploration
and the fused radiation/occupancy map builds live in RViz.

## Known Issues

- **Altitude drift** — the drone's altitude drifts away from its predefined
  setpoint of 1.8 m during flight.
