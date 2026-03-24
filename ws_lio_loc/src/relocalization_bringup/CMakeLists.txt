cmake_minimum_required(VERSION 3.8)
project(relocalization_bringup)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

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

# Install executable
install(TARGETS consolidate_map
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
