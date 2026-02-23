cmake_minimum_required(VERSION 3.8)
project(husky_mission_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(husky_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(mission_bt_node
  src/mission_bt_node.cpp
)

ament_target_dependencies(mission_bt_node
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3
  nav2_msgs
  geometry_msgs
  husky_msgs
  std_srvs
)
target_link_libraries(mission_bt_node
  yaml-cpp
)

install(TARGETS
  mission_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
