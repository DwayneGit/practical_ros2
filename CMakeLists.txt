cmake_minimum_required(VERSION 3.8)
project(practical)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(gotox src/go_to_x.cpp)
add_executable(diff_drive src/simple_diff_drive.cpp)
add_executable(drive_controller src/simple_drive_controller.cpp)
ament_target_dependencies(gotox rclcpp std_msgs geometry_msgs turtlesim)
ament_target_dependencies(diff_drive rclcpp std_msgs geometry_msgs)
ament_target_dependencies(drive_controller rclcpp geometry_msgs nav_msgs)

install(TARGETS
  gotox
  diff_drive
  drive_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
