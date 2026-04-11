cmake_minimum_required(VERSION 3.8)
project(dicerox_urdf_v1)

find_package(ament_cmake REQUIRED)

install(
  FILES launch/display.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING
  PATTERN "*.urdf"
  PATTERN "*.xacro"
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  FILES_MATCHING
  PATTERN "*.rviz"
)

install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
  FILES_MATCHING
  PATTERN "*.stl"
  PATTERN "*.STL"
)

ament_package()
