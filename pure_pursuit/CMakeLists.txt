cmake_minimum_required(VERSION 3.10)
project(pure_pursuit)

# Use C++17 standard
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

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
  ament_index_cpp
  geometry_msgs
  nav_msgs
  ackermann_msgs
  visualization_msgs
)

target_link_libraries(pure_pursuit_node
  Eigen3::Eigen
)

install(TARGETS pure_pursuit_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  waypoints
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pure_pursuit_core test/test_pure_pursuit_core.cpp)
  target_link_libraries(test_pure_pursuit_core Eigen3::Eigen)
endif()

ament_package()
