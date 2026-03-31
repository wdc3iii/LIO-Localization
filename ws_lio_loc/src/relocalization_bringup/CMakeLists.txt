cmake_minimum_required(VERSION 3.8)
project(relocalization_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)

# Add consolidate_map executable
add_executable(consolidate_map
  relocalization_bringup/consolidate_map.cpp
)

# Link dependencies
target_link_libraries(consolidate_map
  ${PCL_LIBRARIES}
  yaml-cpp
)

# Include directories
target_include_directories(consolidate_map PRIVATE
  ${PCL_INCLUDE_DIRS}
)

# C++ standard
target_compile_features(consolidate_map PRIVATE cxx_std_17)

# Add waypoint_dropper executable
add_executable(waypoint_dropper
  relocalization_bringup/waypoint_dropper.cpp
)

target_link_libraries(waypoint_dropper
  ${PCL_LIBRARIES}
  yaml-cpp
)

target_include_directories(waypoint_dropper PRIVATE
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(waypoint_dropper
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  pcl_conversions
  tf2
)

target_compile_features(waypoint_dropper PRIVATE cxx_std_17)
target_compile_definitions(waypoint_dropper PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

# Install executables
install(TARGETS consolidate_map waypoint_dropper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  resource
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install bags and pcd as empty directories via marker files
install(FILES
  bags/.gitkeep
  DESTINATION share/${PROJECT_NAME}/bags
)
install(FILES
  pcd/.gitkeep
  DESTINATION share/${PROJECT_NAME}/pcd
)
install(FILES
  outputs/.gitkeep
  DESTINATION share/${PROJECT_NAME}/outputs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
