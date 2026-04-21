#include "detection_visualizer/detection_visualizer_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  auto visualizer_node = std::make_shared<detection_visualizer::DetectionVisualizerNode>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(visualizer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
