cmake_minimum_required(VERSION 3.22)
project(fanuc_m20ia_35m_planning_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cartesian_control_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_definitions(-DEIGEN_MPL2_ONLY)

set(THIS_PACKAGE_DEPENDS
  ament_cmake
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  rosbag2_cpp
  geometry_msgs
  std_msgs
  control_msgs
  moveit_msgs
  tf2_eigen
  Eigen3
  cartesian_control_msgs
  generate_parameter_library
)

# Generate the parameter library
generate_parameter_library(
  planning_node_parameters # cmake target name for the parameter library
  src/planning_node_parameters.yaml # path to input yaml file
)

# Main executable
add_executable(fanuc_m20ia_35m_planning_demo
  src/fanuc_m20ia_35m_planning_demo.cpp
  src/planning_node.cpp
)

ament_target_dependencies(fanuc_m20ia_35m_planning_demo
  ${THIS_PACKAGE_DEPENDS}
)

target_include_directories(fanuc_m20ia_35m_planning_demo
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(fanuc_m20ia_35m_planning_demo
  planning_node_parameters
)

install(TARGETS
  fanuc_m20ia_35m_planning_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(
  ${THIS_PACKAGE_DEPENDS}
)
ament_package()
