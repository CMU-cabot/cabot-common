cmake_minimum_required(VERSION 3.8)
project(cabot_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cabot_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

add_executable(footprint_publisher
  src/footprint_publisher.cpp
)

ament_target_dependencies(footprint_publisher
  rclcpp
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
  diagnostic_updater
)

add_executable(people_vis_node
  src/people_vis_node.cpp
)

ament_target_dependencies(people_vis_node
  rclcpp
  people_msgs
  visualization_msgs
  tf2
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(MAX_LINE_LENGTH 200)
  ament_copyright()
  ament_cpplint(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_lint_cmake()
  ament_uncrustify(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_xmllint()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

add_library(lookup_transform_service SHARED
  src/lookup_transform_service.cpp
)

ament_target_dependencies(lookup_transform_service
  cabot_msgs
  geometry_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
  rclcpp
  rclcpp_components
)

rclcpp_components_register_node(
  lookup_transform_service
  PLUGIN "CaBot::LookupTransformServiceNode"
  EXECUTABLE lookup_transform_service_node
)

# log redirector
add_executable(log_redirector_node
  src/log_redirector.cpp
)

ament_target_dependencies(log_redirector_node
  rclcpp
  rclcpp_components
  rcl_interfaces
)

# three camera throttle
add_executable(three_camera_throttle_node
  src/three_camera_throttle.cpp
)

ament_target_dependencies(three_camera_throttle_node
  rclcpp
  sensor_msgs
  std_msgs
  message_filters
)

install(PROGRAMS
  scripts/footprint_publisher.py
  scripts/map_loader.py
  scripts/people_vis.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  footprint_publisher
  lookup_transform_service
  people_vis_node
  log_redirector_node
  three_camera_throttle_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()
