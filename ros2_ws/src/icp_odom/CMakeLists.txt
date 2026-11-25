cmake_minimum_required(VERSION 3.8)
project(icp_odom LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG" CACHE STRING "Flags used by the C++ compiler during Release builds." FORCE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(pcl_conversions REQUIRED)

add_executable(icp_node src/icp_node.cpp)
target_include_directories(icp_node PUBLIC
  
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(icp_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  icp_node
  "rclcpp"
  "std_msgs"
  "tf2"
  "tf2_ros"
  "tf2_eigen"
  "tf2_geometry_msgs"
  "message_filters"
  "nav_msgs"
  "sensor_msgs"
  "pcl_conversions"
)

target_link_libraries(icp_node ${PCL_LIBRARIES})

install(TARGETS icp_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
  PATTERN "*.launch.xml"
  PATTERN ".svn" EXCLUDE
  PATTERN "CVS" EXCLUDE
  PATTERN ".git" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
