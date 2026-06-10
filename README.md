## Gazebo Fuel 모델 경로 설정

이 프로젝트의 플랫폼카 모델은 repository 안에 포함되어 있음.

src/tugbot_description/models/independent_steer_vehicle/

따라서 플랫폼 카는 별도 다운로드 없이 git pull 후에 빌드하면 사용 가능.

다만 warehouse world에 포함된 창고, 선반, 카트 등의 배경 모델은 Gazebo Fuel cache 경로를 사용.

예를 들어) 

file:///home/dong/.gz/fuel/fuel.ignitionrobotics.org/movai/models/shelf/1

현재 home/dong pc 기준 경로임.

사용자 이름이 kim이면

file:///home/kim/.gz/fuel/fuel.ignitionrobotics.org/movai/models/shelf/1

수정해주면 됌. 수정할 대상 파일은 src/tugbot_description/worlds/warehouse_vehicle.world 임. 

그리고 필요한 주요 fuel 모델은

warehouse
tugbot-charging-station
cart_model_2
shelf_big
shelf
pallet_box_mobile

## <빌드>
cd ~/robot_ws

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash

ros2 launch tugbot_description warehouse_vehicle.launch.py

## <독립주행 차량 스폰 시키기> 
cd robot_ws

source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 launch tugbot_description warehouse_vehicle.launch.py

## <noraml 모드>
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'normal'}" -1

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0}, angular: {z: 0.0}}" -r 10

## <crab 모드>
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'crab'}" -1

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.1}, angular: {z: 0.0}}" -r 10

## <spin 모드>
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'spin'}" -1

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.1}}" -r 10

## <auto 모드>
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'auto'}" -1

auto 모드는 Nav2/DWB에서 들어오는 /cmd_vel을 받아서 normal, spin, crab 중 안정적인 주행 방식을 내부에서 선택함.
기본 설정에서는 localization 안정성을 위해 crab을 자동으로 사용하지 않고 normal/spin 위주로 동작함.

## <tf구조 만들기>
source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 launch tugbot_description tugbot_bridge_tf.launch.py

## <맵 만들기>
source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 launch tugbot_description tugbot_slam.launch.py

## <맵 저장>
source /opt/ros/humble/setup.bash

ros2 run nav2_map_server map_saver_cli -f /home/dong/robot_ws/my_map

## <자율주행 하기>
source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 launch tugbot_description tugbot_nav2.launch.py

ros2 topic echo /cmd_vel

ros2 topic echo /amcl_pose --once

ros2 run tf2_ros tf2_echo map base_footprint

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
    header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      },
      covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
    }
  }"
