cmake_minimum_required(VERSION 3.5)
project(bot_navigation)

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
find_package(nav2_bringup REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
add_library(dynamic_prediction_path_clear SHARED src/prediction_path_clear.cpp)
target_compile_definitions(dynamic_prediction_path_clear PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(dynamic_prediction_path_clear PUBLIC include)
ament_target_dependencies(dynamic_prediction_path_clear nav2_behavior_tree nav_msgs rclcpp sensor_msgs tf2_ros)
add_library(dynamic_prediction_layer SHARED src/prediction_layer.cpp)
target_include_directories(dynamic_prediction_layer PUBLIC include)
ament_target_dependencies(dynamic_prediction_layer nav2_costmap_2d pluginlib rclcpp sensor_msgs tf2_geometry_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d prediction_plugins.xml)
install(TARGETS dynamic_prediction_layer LIBRARY DESTINATION lib)
install(TARGETS dynamic_prediction_path_clear LIBRARY DESTINATION lib)
install(
  DIRECTORY launch param maps config rviz
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_prediction_layer test/test_prediction_layer.cpp)
  target_include_directories(test_prediction_layer PRIVATE include)
  target_link_libraries(test_prediction_layer dynamic_prediction_layer)
  ament_target_dependencies(test_prediction_layer nav2_costmap_2d rclcpp sensor_msgs)
  # Never inject test forecasts into the real running robot's ROS graph.
  set_tests_properties(test_prediction_layer PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=179;RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_prediction_path_clear test/test_prediction_path_clear.cpp)
  target_include_directories(test_prediction_path_clear PRIVATE include)
  target_link_libraries(test_prediction_path_clear dynamic_prediction_path_clear)
  ament_target_dependencies(test_prediction_path_clear nav2_behavior_tree nav_msgs rclcpp sensor_msgs tf2_ros)
  set_tests_properties(test_prediction_path_clear PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=179;RMW_IMPLEMENTATION=rmw_cyclonedds_cpp;ROS_LOCALHOST_ONLY=1")
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
