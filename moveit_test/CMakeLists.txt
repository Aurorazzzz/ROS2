cmake_minimum_required(VERSION 3.8)
project(moveit_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)   # ✅ SUFFISANT pour trajectory_processing sur Humble

# -----------------------------
# Executables
# -----------------------------
add_executable(moveit_essaie src/moveit_essaie.cpp)
ament_target_dependencies(
  moveit_essaie
  moveit_ros_planning_interface
  rclcpp
)

add_executable(triangle src/triangle.cpp)
ament_target_dependencies(
  triangle
  moveit_ros_planning_interface
  rclcpp
  ament_index_cpp
)

add_executable(dessine src/dessine.cpp)
ament_target_dependencies(
  dessine
  moveit_ros_planning_interface
  rclcpp
  ament_index_cpp
)

add_executable(dessine2 src/dessine2.cpp)
ament_target_dependencies(
  dessine2
  moveit_ros_planning_interface
  moveit_core          # ✅ contient iterative_parabolic_time_parameterization
  rclcpp
  ament_index_cpp
)

# -----------------------------
# Install
# -----------------------------
install(TARGETS
  moveit_essaie
  triangle
  dessine
  dessine2
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

# -----------------------------
# Tests / Lint
# -----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
