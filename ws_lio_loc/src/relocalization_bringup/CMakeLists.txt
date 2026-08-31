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

# Add crop_map executable (crop a map PCD to a waypoint route's region)
add_executable(crop_map
  relocalization_bringup/crop_map.cpp
)

target_link_libraries(crop_map
  ${PCL_LIBRARIES}
  yaml-cpp
)

target_include_directories(crop_map PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_compile_features(crop_map PRIVATE cxx_std_17)
target_compile_definitions(crop_map PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

# Add ply_to_pcd executable (PLY -> PCD converter for GLIM exports)
add_executable(ply_to_pcd
  relocalization_bringup/ply_to_pcd.cpp
)

target_link_libraries(ply_to_pcd
  ${PCL_LIBRARIES}
)

target_include_directories(ply_to_pcd PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_compile_features(ply_to_pcd PRIVATE cxx_std_17)

# Optional: glim_optimize_export — headless equivalent of offline_viewer's
# Optimize + Export Points menu items. Only built when GLIM is available
# (i.e. when the image was built with WITH_GLIM or WITH_GLIM_CUDA).
find_package(glim QUIET)
set(_optional_targets "")
if(glim_FOUND)
  add_executable(glim_optimize_export
    relocalization_bringup/glim_optimize_export.cpp
  )
  target_link_libraries(glim_optimize_export glim::glim)
  target_compile_features(glim_optimize_export PRIVATE cxx_std_17)
  list(APPEND _optional_targets glim_optimize_export)
  message(STATUS "glim found — building glim_optimize_export")
else()
  message(STATUS "glim not found — skipping glim_optimize_export")
endif()

# Install executables
install(TARGETS consolidate_map waypoint_dropper crop_map ply_to_pcd ${_optional_targets}
  DESTINATION lib/${PROJECT_NAME}
)

# Install GLIM helper Python scripts as ros2-runnable executables
install(PROGRAMS
  glim_tools/livox_custom_to_pc2.py
  glim_tools/run_glim_bag.py
  glim_tools/run_glim_chunked.py
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
