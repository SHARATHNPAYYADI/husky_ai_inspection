cmake_minimum_required(VERSION 3.8)
project(husky_mission_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(husky_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(mission_interface_node
  src/mission_interface_node.cpp
)

ament_target_dependencies(mission_interface_node
  rclcpp
  geometry_msgs
  husky_msgs
  std_srvs
)

target_link_libraries(mission_interface_node
  yaml-cpp
)

install(TARGETS
  mission_interface_node
  DESTINATION lib/${PROJECT_NAME}
)


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
