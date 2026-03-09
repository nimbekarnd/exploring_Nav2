cmake_minimum_required(VERSION 3.8)
project(nav2_custom_costmap_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

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

add_library(nav2_custom_costmap_plugin SHARED
src/custom_inflation_layer.cpp
)

target_include_directories(nav2_custom_costmap_plugin PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_custom_costmap_plugin
rclcpp
nav2_costmap_2d
pluginlib
)

pluginlib_export_plugin_description_file(
  nav2_costmap_2d costmap_plugins.xml
)
  
install(TARGETS nav2_custom_costmap_plugin
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)

install(DIRECTORY include/
DESTINATION include/
)

install(FILES costmap_plugins.xml
DESTINATION share/${PROJECT_NAME}
)
  
ament_package()