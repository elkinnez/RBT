cmake_minimum_required(VERSION 3.8)
project(learning_tf2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

# === BROADCASTER (turtle1) ===
add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(turtle_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)

# === LISTENER (turtle2 sigue a turtle1) ===
add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
ament_target_dependencies(turtle_tf2_listener
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)

# === NUEVO: BROADCASTER DINÁMICO (carrot1) ===
add_executable(dynamic_frame_tf2_broadcaster src/dynamic_frame_tf2_broadcaster.cpp)
ament_target_dependencies(dynamic_frame_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

# Install all executables
install(TARGETS
  turtle_tf2_broadcaster
  turtle_tf2_listener
  dynamic_frame_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (si tienes la carpeta launch)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()