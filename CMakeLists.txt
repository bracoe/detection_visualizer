cmake_minimum_required(VERSION 3.10)
project(detection_visualizer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)


add_library(detection_visualizer_node SHARED
  src/detection_visualizer_node.cpp)
target_compile_definitions(detection_visualizer_node PRIVATE "DETECTION_VISUALIZER_NODE_BUILDING_DLL")
target_include_directories(detection_visualizer_node PUBLIC include)
ament_target_dependencies(detection_visualizer_node
  rclcpp
  rclcpp_components
  cv_bridge
  sensor_msgs
  vision_msgs
  image_transport)

rclcpp_components_register_nodes(detection_visualizer_node "detection_visualizer::DetectionVisualizerNode")

add_executable(detection_visualizer_node_main
  src/detection_visualizer_node_main.cpp)
target_link_libraries(detection_visualizer_node_main detection_visualizer_node)
set_target_properties(detection_visualizer_node_main PROPERTIES OUTPUT_NAME "detection_visualizer")

#No tests implemented yet.
#if(BUILD_TESTING)
#  
#endif()

install(TARGETS detection_visualizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS detection_visualizer_node_main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/ launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)

ament_package()
