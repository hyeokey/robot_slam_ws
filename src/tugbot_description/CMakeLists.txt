cmake_minimum_required(VERSION 3.8)
project(tugbot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
