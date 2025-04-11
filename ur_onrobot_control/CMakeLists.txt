cmake_minimum_required(VERSION 3.8)
project(ur_onrobot_control)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
