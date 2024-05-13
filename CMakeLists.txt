cmake_minimum_required(VERSION 3.5)
project(floatgen)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf launch DESTINATION share/${PROJECT_NAME})

ament_package()
