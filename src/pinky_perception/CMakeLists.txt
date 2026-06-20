cmake_minimum_required(VERSION 3.8)
project(pinky_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# NOTE: pcl_ros intentionally NOT required — scan_fuser uses raw PCL (linked below via
# find_package(PCL)) + pcl_conversions only, never any pcl_ros API. Requiring pcl_ros
# broke the build on the robot where ros-jazzy-pcl-ros isn't installed (isolated AP).
find_package(PCL REQUIRED COMPONENTS common filters segmentation)

add_executable(scan_fuser src/scan_fuser_node.cpp)
target_include_directories(scan_fuser PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(scan_fuser ${PCL_LIBRARIES})
ament_target_dependencies(scan_fuser
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions)

install(TARGETS scan_fuser
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
