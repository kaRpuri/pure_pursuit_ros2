cmake_minimum_required(VERSION 3.5)
project(pure_pursuit)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(pure_pursuit_node 
  src/pure_pursuit_node.cpp
)


ament_target_dependencies(pure_pursuit_node
  rclcpp
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  visualization_msgs
  tf2_ros
)


target_link_libraries(pure_pursuit_node
  Eigen3::Eigen
)


install(TARGETS pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)


install(FILES 
  src/levine2.csv
  DESTINATION share/${PROJECT_NAME}
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
