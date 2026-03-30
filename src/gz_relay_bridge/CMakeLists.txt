cmake_minimum_required(VERSION 3.8)
project(gz_relay_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

add_executable(gz_relay_bridge_node src/gz_relay_bridge_node.cpp)
ament_target_dependencies(gz_relay_bridge_node
  rclcpp sensor_msgs nav_msgs tf2_msgs tf2_ros geometry_msgs rosgraph_msgs builtin_interfaces
)

add_executable(independent_steer_controller_node src/independent_steer_controller_node.cpp)
ament_target_dependencies(independent_steer_controller_node
  rclcpp geometry_msgs std_msgs
)

find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

target_link_libraries(gz_relay_bridge_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

target_link_libraries(independent_steer_controller_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS gz_relay_bridge_node
  independent_steer_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
