cmake_minimum_required(VERSION 3.5)
project(ur5e_l515_description)

find_package(ament_cmake REQUIRED)

foreach(dir meshes urdf worlds config launch)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

ament_package()
