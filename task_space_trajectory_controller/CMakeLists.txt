cmake_minimum_required(VERSION 3.5)
project(task_space_trajectory_controller)

set(CMAKE_CXX_STANDARD 17)
set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_definitions(-DEIGEN_MPL2_ONLY)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(cartesian_control_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdf REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(interactive_markers REQUIRED)

# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        rclcpp_action
        pluginlib
        controller_interface
        cartesian_control_msgs
        realtime_tools
        urdf
        Eigen3
        moveit_core
        moveit_ros_planning
        geometry_msgs
        generate_parameter_library
        interactive_markers
)

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

generate_parameter_library(
  ${PROJECT_NAME}_parameters # cmake target name for the parameter library
  src/task_space_trajectory_controller_parameters.yaml # path to input yaml file
)

generate_parameter_library(
  reference_generator_parameters # cmake target name for the parameter library
  src/reference_generator_parameters.yaml # path to input yaml file
)

# ---------------------- Library target
add_library(${PROJECT_NAME} SHARED
  src/task_space_trajectory_controller.cpp
  src/task_space_trajectory_interpolator.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${PROJECT_NAME}_parameters
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Export as a plugin
pluginlib_export_plugin_description_file(controller_interface task_space_trajectory_controller_plugin.xml)

# ---------------------- Reference generator target
add_executable(reference_generator
  src/reference_generator.cpp
)

target_link_libraries(reference_generator PUBLIC
  reference_generator_parameters
)

ament_target_dependencies(reference_generator PUBLIC
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# The following lines are commented out because they cause unexpected behavior
#ament_export_targets(my_targets_from_this_package HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  #EXPORT my_targets_from_this_package
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  #INCLUDES DESTINATION include
)

install(TARGETS reference_generator DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
