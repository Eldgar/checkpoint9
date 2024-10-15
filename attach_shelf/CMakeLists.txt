cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

# Add the GoToLoading service file
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
  DEPENDENCIES std_msgs
)

# Add the executables
add_executable(pre_approach src/pre_approach.cpp)
add_executable(pre_approach_v2 src/pre_approach_v2.cpp)
add_executable(leg_detection src/leg_detection.cpp)
add_executable(approach_service_server src/approach_service_server.cpp)

# Specify dependencies for each executable
ament_target_dependencies(pre_approach
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(pre_approach_v2
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  std_msgs
)

ament_target_dependencies(leg_detection
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

ament_target_dependencies(approach_service_server
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
)

# Ensure that the service messages are built before the executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_dependencies(pre_approach_v2 ${cpp_typesupport_target})
add_dependencies(leg_detection ${cpp_typesupport_target})
add_dependencies(approach_service_server ${cpp_typesupport_target})

# Link the service dependencies
rosidl_target_interfaces(pre_approach_v2 ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(leg_detection ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(approach_service_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the executables
install(TARGETS
  pre_approach
  pre_approach_v2
  leg_detection
  approach_service_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # Comment the line when you have added a copyright
  # and license to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

