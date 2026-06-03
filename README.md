## <독립주행 차량 스폰 시키기> 
cd robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tugbot_description warehouse_vehicle.launch.py

base_footprint
odom
base_link
laser
scan_omni

ros2 topic echo /plan --once
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /particle_cloud --once
## <noraml 모드>
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'normal'}" -1
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0}, angular: {z: 0.0}}" -r 10

## <crab 모드>
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'crab'}" -1
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.1}, angular: {z: 0.0}}" -r 10

## <spin 모드>
ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'spin'}" -1
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0}, angular: {z: 0.1}}" -r 10

## <tf구조 만들기
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tugbot_description tugbot_bridge_tf.launch.py

## <맵 만들기>
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tugbot_description tugbot_slam.launch.py

## <맵 저장>
cd /home/dong/robot_ws
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f /home/dong/robot_ws/my_map

## <자율주행 하기>
cd /home/dong/robot_ws
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
