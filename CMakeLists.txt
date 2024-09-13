cmake_minimum_required(VERSION 3.5)
project(floatgen)

find_package(ament_cmake REQUIRED)

ament_environment_hooks("hooks/resource_paths.dsv.in")
install(DIRECTORY meshes urdf launch DESTINATION share/${PROJECT_NAME})

ament_package()
