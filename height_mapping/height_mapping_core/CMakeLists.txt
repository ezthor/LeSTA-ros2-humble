# Modified for LeSTA ROS 2 Humble: export the height mapping library through ament.
cmake_minimum_required(VERSION 3.8)
project(height_mapping_core)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
add_library(${PROJECT_NAME} SHARED
  src/HeightMap.cpp src/KalmanEstimator.cpp src/MovingAverageEstimator.cpp
  src/StatMeanEstimator.cpp src/FastHeightFilter.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} grid_map_core)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(grid_map_core)
ament_package()
