#include "detection_visualizer/detection_visualizer_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto visualizer_node = std::make_shared<detection_visualizer::DetectionVisualizerNode>(options);
  
  rclcpp::spin(visualizer_node);

  rclcpp::shutdown();
  return 0;
}