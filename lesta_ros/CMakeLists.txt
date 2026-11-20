# Modified for LeSTA ROS 2 Humble: build and install the ament runtime.
cmake_minimum_required(VERSION 3.8)
project(lesta)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lesta_interfaces REQUIRED)
find_package(height_mapping_core REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(Torch REQUIRED)
add_executable(lesta_node
  src/ros2/lesta_node.cpp
  src/core/HeightMapper.cpp src/core/GlobalMapper.cpp
  src/core/FeatureExtractor.cpp src/core/LabelGenerator.cpp
  src/core/TraversabilityNetwork.cpp src/core/TraversabilityEstimator.cpp
  src/core/TraversabilityMapper.cpp)
target_include_directories(lesta_node PRIVATE include ${PCL_INCLUDE_DIRS} ${TORCH_INCLUDE_DIRS})
ament_target_dependencies(lesta_node
  rclcpp sensor_msgs geometry_msgs grid_map_core grid_map_ros grid_map_msgs
  tf2_ros tf2_eigen pcl_conversions visualization_msgs
  height_mapping_core lesta_interfaces)
target_link_libraries(lesta_node ${PCL_LIBRARIES} ${TORCH_LIBRARIES})
target_compile_options(lesta_node PRIVATE ${TORCH_CXX_FLAGS})
install(TARGETS lesta_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config/ros2 DESTINATION share/${PROJECT_NAME})
ament_package()
