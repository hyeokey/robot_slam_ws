# robot_slam_ws

Tugbot 시뮬레이션, SLAM, Nav2 테스트를 위한 ROS 2 Humble 워크스페이스입니다. Gazebo Sim 환경에서 맵 생성과 저장된 맵 기반 주행을 할 수 있도록 구성되어 있습니다.

## 구성

- `src/gz_relay_bridge`
  - Gazebo Sim 토픽을 ROS 2 토픽으로 연결하는 브리지 패키지입니다.
  - 라이다, 오도메트리, 시뮬레이션 시간, `/cmd_vel`을 처리합니다.
- `src/tugbot_description`
  - Tugbot URDF, launch 파일, Nav2 파라미터, SLAM 파라미터를 포함합니다.
- `my_map.yaml`, `my_map.pgm`
  - 저장된 맵 파일입니다.

## 주요 노드

### `gz_relay_bridge`

- `gz_relay_bridge_node`
  - Gazebo의 스캔, 오도메트리, 클럭 정보를 ROS 2로 전달합니다.
  - `odom -> base_footprint` TF를 발행합니다.
  - 시작 시점 기준으로 오도메트리를 0으로 맞출 수 있습니다.
- `independent_steer_controller_node`
  - Gazebo 안의 Tugbot 모델을 제어하는 노드입니다.

### `tugbot_description`

- 로봇 모델: `urdf/tugbot_minimal.urdf.xacro`
- 브리지 실행: `launch/tugbot_bridge_tf.launch.py`
- SLAM 실행: `launch/tugbot_slam.launch.py`
- Nav2 실행: `launch/tugbot_nav2.launch.py`
- Gazebo 월드 실행: `launch/warehouse_vehicle.launch.py`

## 실행 환경

기준 환경:

- Ubuntu 22.04
- ROS 2 Humble
- `gz` 명령을 사용할 수 있는 Gazebo Sim 환경

필요한 ROS 2 패키지 예시:

- `nav2_bringup`
- `slam_toolbox`
- `robot_state_publisher`
- `xacro`

또한 `gz_relay_bridge` 빌드를 위해 Gazebo transport, message 라이브러리가 필요합니다.

## 빌드

```bash
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행 순서

### 1. Gazebo 월드 실행

```bash
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tugbot_description warehouse_vehicle.launch.py
```

이 launch는 다음을 실행합니다.

- `warehouse_vehicle.world`
- Tugbot 제어 노드

### 2. 브리지와 SLAM 실행

새 터미널에서 실행:

```bash
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tugbot_description tugbot_slam.launch.py
```

이 launch는 다음을 실행합니다.

- `robot_state_publisher`
- `gz_relay_bridge_node`
- `slam_toolbox` 온라인 비동기 매핑

### 3. 생성한 맵 저장

매핑이 끝난 뒤 실행:

```bash
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 4. 저장된 맵으로 Nav2 실행

새 터미널에서 실행:

```bash
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tugbot_description tugbot_nav2.launch.py
```

기본 맵 파일 경로:

- `/home/dong/robot_ws/my_map.yaml`

다른 맵을 쓰려면 다음처럼 지정하면 됩니다.

```bash
ros2 launch tugbot_description tugbot_nav2.launch.py map:=/home/dong/robot_ws/my_map.yaml
```

## 주요 토픽과 프레임

주요 토픽:

- `/scan`
- `/odom`
- `/cmd_vel`
- `/clock`

주요 프레임:

- `map`
- `odom`
- `base_footprint`
- `scan_omni`

## 주의사항

- `warehouse_vehicle.launch.py` 안에는 `/home/dong/Desktop`, `/home/dong/.gz/fuel/...` 같은 리소스 경로가 하드코딩되어 있습니다.
- `tugbot_nav2.launch.py`의 기본 맵 경로는 `/home/dong/robot_ws/my_map.yaml`로 고정되어 있습니다.
- 다른 PC나 다른 사용자 계정에서 실행할 경우 위 경로들은 수정이 필요할 수 있습니다.

## 저장소

GitHub:

- <https://github.com/hyeokey/robot_slam_ws>
