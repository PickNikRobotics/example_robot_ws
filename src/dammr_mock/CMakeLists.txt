cmake_minimum_required(VERSION 3.22)
project(dammr_mock)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes objectives urdf waypoints DESTINATION share/${PROJECT_NAME}/)

ament_package()
