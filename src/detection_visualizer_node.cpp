#include "detection_visualizer/detection_visualizer_node.hpp"
#include <cmath>
#include <sstream>

namespace detection_visualizer
{

DetectionVisualizerNode::DetectionVisualizerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("DetectionVisualizerNode", options)
{

  // TransportHints does not actually declare the parameter
  this->declare_parameter<std::string>("image_transport", "raw");
  int queue_size = this->declare_parameter<int>("queue_size", 20);

  synchronizer_ = std::make_shared<ApproxSynchronizer>(ApproxSyncPolicy(queue_size), sub_image_, sub_detections_);
    synchronizer_->registerCallback(
      std::bind(
        &DetectionVisualizerNode::syncCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

  // For compressed topics to remap appropriately, we need to pass a
  // fully expanded and remapped topic name to image_transport
  auto node_base = this->get_node_base_interface();
  std::string topic = node_base->resolve_topic_or_service_name("image_raw", false);
  image_transport::TransportHints hints(this, "raw", "image_transport");
  sub_image_.subscribe(this, topic, hints.getTransport(), rmw_qos_profile_sensor_data);
  sub_detections_.subscribe(this, "detections");

  //Publisher
  node_base = this->get_node_base_interface();
  topic = node_base->resolve_topic_or_service_name("dbg_image", false);
  pub_image_ = image_transport::create_publisher(this, topic);

}

void DetectionVisualizerNode::syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections_msg){

  cv::Mat cv_image = cv_bridge::toCvShare(image_msg)->image;
  int color_count = 0;
  for (auto &detection: detections_msg->detections)
  {
    std::string max_class;
    double max_score = 0;
    for (auto &result: detection.results){
      if (result.hypothesis.score > max_score){
        max_score = result.hypothesis.score;
        max_class = result.hypothesis.class_id;
      }

      if (max_class.empty()){
        RCLCPP_WARN(this->get_logger(), "Failed to find class with highest score!");
        return;
      }

    }

    double center_x = detection.bbox.center.position.x;
    double center_y = detection.bbox.center.position.y;
    double size_x = detection.bbox.size_x;
    double size_y = detection.bbox.size_y;

    auto min_point = cv::Point2d(std::rint(center_x - size_x/2), std::rint(center_y - size_y/2));
    auto max_point = cv::Point2d(std::rint(center_x + size_x/2), std::rint(center_y + size_y/2));

    //Draw rectangle
    auto loop_color = colors.at(color_count++%colors.size());
    int thickness = 1; 
    cv::rectangle(cv_image, min_point, max_point, 
            loop_color, thickness*2, cv::LINE_8); 

    //Add name and score
    std::string label;
    label.append(max_class);
    label.append(": ");
    std::stringstream stream;
    stream << std::fixed << std::setprecision(2) << max_score;
    label.append(stream.str());
    auto text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.75, thickness, nullptr);

    auto position = cv::Point2d(min_point.x, min_point.y);
    cv::rectangle(cv_image, position, cv::Point2d(position.x + text_size.width, position.y - text_size.height), loop_color, -1);
    cv::putText(cv_image, label, position, cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA);

  }
  
  sensor_msgs::msg::Image::SharedPtr drawn_image_msg = cv_bridge::CvImage(image_msg->header, image_msg->encoding, cv_image).toImageMsg();
  pub_image_.publish(std::move(drawn_image_msg));
}


} // namespace detection_visualizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detection_visualizer::DetectionVisualizerNode)
