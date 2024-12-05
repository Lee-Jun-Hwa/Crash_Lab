cmake_minimum_required(VERSION 3.5)
project(tutorial_ros2_motor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)



include_directories(
  include
  ${ament_index_cpp_INCLUDE_DIRS}
  )


add_executable(test_motor_node src/test_motor_node.cpp)
target_link_libraries(test_motor_node pigpiod_if2)
ament_target_dependencies(test_motor_node rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs ${ament_index_cpp_LIBRARIES})


add_executable(twist_publisher src/twist_publisher.cpp)
ament_target_dependencies(twist_publisher rclcpp geometry_msgs)

add_executable(teleop_node src/teleop_node.cpp)
ament_target_dependencies(teleop_node rclcpp geometry_msgs)

install(TARGETS
  test_motor_node
  teleop_node
  twist_publisher
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
