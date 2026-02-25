cmake_minimum_required(VERSION 3.22)
project(example_robot_sim)

find_package(ament_cmake REQUIRED)

# Install configuration files
install(DIRECTORY config mjcf launch urdf DESTINATION share/${PROJECT_NAME}/)

ament_package()
