
#include <cstddef>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
#include <image_tools/cv_mat_sensor_msgs_image_type_adapter.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <cv_bridge/cv_bridge.h>

#include "videocapture.hpp"
#include "videocapture_def.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(image_tools::ROSCvMatContainer, sensor_msgs::msg::Image);

class ZedCam2ImageNode : public rclcpp::Node {
 public:
  explicit ZedCam2ImageNode(const rclcpp::NodeOptions &options) : Node("zed_cam2image", options) {
    parse_parameters();

    // Create Video Capture
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_60;
    zed_video_capture_ = std::make_unique<sl_oc::video::VideoCapture>(params);
    if (!zed_video_capture_->initializeVideo()) {
      RCLCPP_ERROR(this->get_logger(), "annot open camera video capture. See verbosity level for more details.");
      throw std::runtime_error("Could not open video stream");
    }
    RCLCPP_INFO(
      this->get_logger(), "Connected to camera sn: %d [%s]", zed_video_capture_->getSerialNumber(),
      zed_video_capture_->getDeviceName().c_str()
    );

    // Create ROS publishers based on whether the node should combine both cameras to the same image (stereo mode) or
    // separate images (left and right)
    if (stereo_mode_) {
      image_stereo_pub_ = create_publisher<image_tools::ROSCvMatContainer>("image_stereo", 10);
    } else {
      image_left_pub_ = create_publisher<image_tools::ROSCvMatContainer>("image_left", 10);
      image_right_pub_ = create_publisher<image_tools::ROSCvMatContainer>("image_right", 10);
    }

    // Start main timer loop
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)), [this]() {
      return this->timerCallback();
    });
  }

 private:
  void timerCallback() {
    // Get the last available frame from the video capture
    const sl_oc::video::Frame frame = zed_video_capture_->getLastFrame();

    // If no frame was grabbed, return early
    if (frame.data == nullptr) {
      return;
    }

    // Create image from the cv_bridge package to easily convert between OpenCV image and ROS 2 image
    auto cv_image_stereo = cv_bridge::CvImage();

    std_msgs::msg::Header header;
    header.stamp = this->now();  // TODO: Obtain time stamp from frame.timestamp instead (might reduce processing delay)
    header.frame_id = frame_id_;
    cv_image_stereo.header = header;

    // Conversion from YUV 4:2:2 to RGB
    cv::Mat frame_yuv = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
    cv::cvtColor(frame_yuv, cv_image_stereo.image, cv::COLOR_YUV2RGB_YUYV);
    cv_image_stereo.encoding = "rgb8";

    // Convert to ROS message and publish
    publish_number_++;
    if (stereo_mode_) {
      RCLCPP_INFO(get_logger(), "Publishing stereo image #%zd", publish_number_);
      sensor_msgs::msg::Image::SharedPtr ros_image = cv_image_stereo.toImageMsg();
      image_stereo_pub_->publish(*ros_image);
      return;  // Return early as we only want to publish the stereo image in stereo mode
    }

    // Split the stereo image into left image
    auto cv_image_left = cv_bridge::CvImage();
    cv_image_left.header = cv_image_stereo.header;
    cv_image_left.encoding = cv_image_stereo.encoding;
    cv_image_left.image = cv_image_stereo.image(cv::Range::all(), cv::Range(0, cv_image_stereo.image.cols / 2));
    sensor_msgs::msg::Image::SharedPtr ros_image_left = cv_image_left.toImageMsg();

    // Split the stereo image into right image
    auto cv_image_right = cv_bridge::CvImage();
    cv_image_right.header = cv_image_stereo.header;
    cv_image_right.encoding = cv_image_stereo.encoding;
    cv_image_right.image =
      cv_image_stereo.image(cv::Range::all(), cv::Range(cv_image_stereo.image.cols / 2, cv_image_stereo.image.cols));
    sensor_msgs::msg::Image::SharedPtr ros_image_right = cv_image_right.toImageMsg();

    // Publish the left and right messages to ROS
    RCLCPP_INFO(get_logger(), "Publishing left and right images #%zd", publish_number_);
    image_left_pub_->publish(*ros_image_left);
    image_right_pub_->publish(*ros_image_right);
  }

  void parse_parameters() {
    freq_ = this->declare_parameter("frequency", 30.0);
    frame_id_ = this->declare_parameter("frame_id", "camera_frame");
    frame_id_ = this->declare_parameter("stereo_mode", false);
  }

 private:
  rclcpp::Publisher<image_tools::ROSCvMatContainer>::SharedPtr image_stereo_pub_;
  rclcpp::Publisher<image_tools::ROSCvMatContainer>::SharedPtr image_left_pub_;
  rclcpp::Publisher<image_tools::ROSCvMatContainer>::SharedPtr image_right_pub_;
  rclcpp::TimerBase::SharedPtr                                 timer_;

  std::unique_ptr<sl_oc::video::VideoCapture> zed_video_capture_;

  // ROS parameters
  double      freq_;
  std::string frame_id_;
  bool        stereo_mode_;

  /// The number of images published.
  size_t publish_number_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::cout << "Hello from zed_cam2image" << std::endl;
  rclcpp::NodeOptions node_options;
  rclcpp::spin(std::make_shared<ZedCam2ImageNode>(node_options));
  rclcpp::shutdown();
  return 0;
}