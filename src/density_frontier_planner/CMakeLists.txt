cmake_minimum_required(VERSION 3.1)
project(density_frontier_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

# Intentionally dependency-light: roscpp + standard messages only.
# No PCL, no Eigen, no OR-Tools, no tf2. PointCloud2 x/y/z are decoded by hand.
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs std_msgs visualization_msgs
)

include_directories(
  include
  SYSTEM ${catkin_INCLUDE_DIRS}
)

add_library(dfp_core
  src/occupancy_grid.cpp
  src/density_target_manager.cpp
  src/viewpoint_planner.cpp
)
add_dependencies(dfp_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dfp_core ${catkin_LIBRARIES})

add_executable(density_frontier_planner_node
  src/planner_node.cpp
  src/planner_main.cpp
)
add_dependencies(density_frontier_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(density_frontier_planner_node dfp_core ${catkin_LIBRARIES})

install(TARGETS dfp_core density_frontier_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
