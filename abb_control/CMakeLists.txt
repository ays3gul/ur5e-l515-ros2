cmake_minimum_required(VERSION 3.8)
project(abb_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# Some MoveIt deps (fmt, tl-expected) are pulled from conda, which adds
# miniconda/lib to RPATH. Prepend the system lib so its OpenSSL is found first.
set(CMAKE_EXE_LINKER_FLAGS
  "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath-link,/usr/lib/x86_64-linux-gnu -L/usr/lib/x86_64-linux-gnu")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(abb_interfaces REQUIRED)

# Library: arm_control (MoveIt wrapper)
add_library(${PROJECT_NAME}
  src/arm_control.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  abb_interfaces
)

# Executable: arm_control_node
add_executable(arm_control_node src/arm_control_node.cpp)
target_link_libraries(arm_control_node ${PROJECT_NAME})
ament_target_dependencies(arm_control_node
  rclcpp
  geometry_msgs
  abb_interfaces
)

install(TARGETS ${PROJECT_NAME} arm_control_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
