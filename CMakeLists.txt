cmake_minimum_required(VERSION 3.8)
project(chess_player)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(chess_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

generate_parameter_library(
  chess_player_params
  src/parameters.yaml
)

add_subdirectory(libchess)

add_executable(chess_player
  src/main.cpp
  src/chess_player_node.cpp
  src/motion.cpp
  src/play_game.cpp
)

target_include_directories(chess_player PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(chess_player PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20

ament_target_dependencies(chess_player PUBLIC
  moveit_ros_planning_interface
  moveit_servo
  chess_msgs
  sensor_msgs
  rclcpp_action
)

target_link_libraries(chess_player PRIVATE
  chess_player_params
  libchess_static
)

install(TARGETS chess_player
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
