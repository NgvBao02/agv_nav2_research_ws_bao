cmake_minimum_required(VERSION 3.16)
project(adaptive_pivot_g2_nav2 VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(adaptive_pivot_g2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_smoother REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/adaptive_pivot_g2_smoother.cpp
  src/footprint_safety.cpp
  src/safety_gated_hybrid_smoother.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  adaptive_pivot_g2
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_smoother
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_ros
)

pluginlib_export_plugin_description_file(nav2_core adaptive_pivot_g2_nav2_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES adaptive_pivot_g2_nav2_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_footprint_safety test/test_footprint_safety.cpp)
  target_compile_features(test_footprint_safety PRIVATE cxx_std_17)
  target_link_libraries(test_footprint_safety ${PROJECT_NAME})
  ament_target_dependencies(
    test_footprint_safety geometry_msgs nav2_costmap_2d)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  adaptive_pivot_g2 nav2_core nav2_costmap_2d nav2_smoother nav_msgs pluginlib rclcpp_lifecycle
)
ament_package()
