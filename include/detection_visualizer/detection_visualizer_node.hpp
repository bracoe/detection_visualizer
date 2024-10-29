#ifndef DETECTION_VISUALIZER_NODE_HPP_
#define DETECTION_VISUALIZER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>


namespace detection_visualizer
{

class DetectionVisualizerNode : public rclcpp::Node
{
public:
  explicit DetectionVisualizerNode(const rclcpp::NodeOptions & options);

private:
  image_transport::SubscriberFilter sub_image_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> sub_detections_;
  using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>;
  using ApproxSynchronizer = message_filters::Synchronizer<ApproxSyncPolicy>;
  std::shared_ptr<ApproxSynchronizer> synchronizer_;

  std::mutex connect_mutex_;
  image_transport::Publisher pub_image_;

  //Should be enough to give all detections a unique color in the image
  std::vector<cv::Scalar> colors = {
    cv::Scalar(255,105,180),
    cv::Scalar(255,0,0),
    cv::Scalar(255,165,0),
    cv::Scalar(255,255,0),
    cv::Scalar(0,128,0),
    cv::Scalar(64,224,208),
    cv::Scalar(75,0,130),
    cv::Scalar(238,130,238),
    /*cv::Scalar(0, 0, 128),
    cv::Scalar(0, 128, 128),
    cv::Scalar(128, 0, 128),
    cv::Scalar(128, 128, 0)*/
  };

  void syncCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections_msg);

  cv::Scalar getColor(int loop);
  
};


}  // namespace detection_visualizer

#endif  // DETECTION_VISUALIZER_NODE_HPP_