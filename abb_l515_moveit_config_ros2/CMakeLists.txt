cmake_minimum_required(VERSION 3.8)
project(abb_l515_moveit_config_ros2)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
