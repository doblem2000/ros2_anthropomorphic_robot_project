cmake_minimum_required(VERSION 3.22)
project(acg_resources_fanuc_m20ia_35m_description)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf launch config DESTINATION share/${PROJECT_NAME})