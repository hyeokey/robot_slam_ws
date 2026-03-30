# robot_slam_ws

ROS 2 Humble workspace for Tugbot simulation, SLAM, and Nav2 testing with Gazebo Sim.

## Workspace layout

- `src/gz_relay_bridge`: Gazebo Sim to ROS 2 bridge nodes
- `src/tugbot_description`: Tugbot URDF, launch files, Nav2 and SLAM parameters
- `my_map.yaml`, `my_map.pgm`: saved map for Nav2

## Main components

### `gz_relay_bridge`

- `gz_relay_bridge_node`
  - relays Gazebo scan, odometry, clock, and `/cmd_vel`
  - publishes `odom -> base_footprint` TF
  - can zero odometry at startup
- `independent_steer_controller_node`
  - drives the Tugbot model inside Gazebo Sim

### `tugbot_description`

- robot description: `urdf/tugbot_minimal.urdf.xacro`
- bridge launch: `launch/tugbot_bridge_tf.launch.py`
- SLAM launch: `launch/tugbot_slam.launch.py`
- Nav2 launch: `launch/tugbot_nav2.launch.py`
- Gazebo world launch: `launch/warehouse_vehicle.launch.py`

## Environment

Tested for:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim Harmonic or compatible `gz` command environment

Required ROS 2 packages include:

- `nav2_bringup`
- `slam_toolbox`
- `robot_state_publisher`
- `xacro`

Build dependencies also include Gazebo transport and message libraries used by `gz_relay_bridge`.

## Build

```bash
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

### 1. Start Gazebo world

```bash
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tugbot_description warehouse_vehicle.launch.py
```

This launches:

- `warehouse_vehicle.world`
- the Tugbot controller node

### 2. Run bridge and SLAM

Open a new terminal:

```bash
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tugbot_description tugbot_slam.launch.py
```

This starts:

- `robot_state_publisher`
- `gz_relay_bridge_node`
- `slam_toolbox` in online async mapping mode

### 3. Save the generated map

After mapping:

```bash
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 4. Run Nav2 with the saved map

Open a new terminal:

```bash
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tugbot_description tugbot_nav2.launch.py
```

Default map file:

- `/home/dong/robot_ws/my_map.yaml`

If needed, override it:

```bash
ros2 launch tugbot_description tugbot_nav2.launch.py map:=/home/dong/robot_ws/my_map.yaml
```

## Topics and frames

Important ROS 2 topics:

- `/scan`
- `/odom`
- `/cmd_vel`
- `/clock`

Important frames:

- `map`
- `odom`
- `base_footprint`
- `scan_omni`

## Notes

- `warehouse_vehicle.launch.py` currently contains hardcoded resource paths under `/home/dong/Desktop` and `/home/dong/.gz/fuel/...`.
- `tugbot_nav2.launch.py` currently defaults to `/home/dong/robot_ws/my_map.yaml`.
- If this workspace is moved to another machine or another user account, those paths should be updated.

## Repository

GitHub:

- <https://github.com/hyeokey/robot_slam_ws>
