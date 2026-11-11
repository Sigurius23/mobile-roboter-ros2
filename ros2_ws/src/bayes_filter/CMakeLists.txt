cmake_minimum_required(VERSION 3.8)
project(bayes_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

# messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WorldUpdate.msg"
)
rosidl_get_typesupport_target(world_update_cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# simulated_world
add_executable(simulated_world src/simulated_world.cpp)
target_include_directories(simulated_world PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(simulated_world PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  simulated_world
  "rclcpp"
  "geometry_msgs"
  "tf2_ros"
)
target_link_libraries(simulated_world "${world_update_cpp_typesupport_target}")
install(TARGETS simulated_world
  DESTINATION lib/${PROJECT_NAME})

# bayes_filter_and_costmap
add_executable(bayes_filter_and_costmap src/bayes_filter_and_costmap.cpp)
target_include_directories(bayes_filter_and_costmap PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(bayes_filter_and_costmap PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  bayes_filter_and_costmap
  "rclcpp"
  "nav_msgs"
)
target_link_libraries(bayes_filter_and_costmap "${world_update_cpp_typesupport_target}")
install(TARGETS bayes_filter_and_costmap
  DESTINATION lib/${PROJECT_NAME})

# RViz config from the sheet: ros2 run rviz2 rviz2 -d $(ros2 pkg prefix bayes_filter)/share/bayes_filter/bayes.rviz
install(FILES bayes.rviz
  DESTINATION share/${PROJECT_NAME})

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
