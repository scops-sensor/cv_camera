// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_CAPTURE_H
#define CV_CAMERA_CAPTURE_H

#include "cv_camera/exception.h"
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

/**
 * @brief namespace of this package
 */
namespace cv_camera
{

/**
 * @brief captures by cv::VideoCapture and publishes to ROS topic.
 *
 */
class Capture
{
public:
  /**
   * @brief costruct with ros node and topic settings
   *
   * @param node ROS node handle for advertise topic.
   * @param topic_name name of topic to publish (this may be image_raw).
   * @param buffer_size size of publisher buffer.
   * @param frame_id frame_id of publishing messages.
   */
  Capture(rclcpp::Node::SharedPtr node,
          const std::string &topic_name,
          uint32_t buffer_size,
          const std::string &frame_id);

  /**
   * @brief Open capture device with device ID.
   *
   * @param device_id id of camera device (number from 0)
   * @throw cv_camera::DeviceError device open failed
   *
   */
  void open(int32_t device_id);

  /**
   * @brief Open capture device with device name.
   *
   * @param device_sn path of the camera device
   * @throw cv_camera::DeviceError device open failed
   */
  void open(const std::string &device_sn);

  /**
   * @brief Load camera info from file.
   *
   * This loads the camera info from the file specified in the camera_info_url parameter.
   */
  void loadCameraInfo();

  /**
   * @brief Open default camera device.
   *
   * This opens with device 0.
   *
   * @throw cv_camera::DeviceError device open failed
   */
  void open();

  /**
   * @brief open video file instead of capture device.
   */
  void openFile(const std::string &file_path);

  /**
   * @brief capture an image and store.
   *
   * to publish the captured image, call publish();
   * @return true if success to capture, false if not captured.
   */
  bool capture();

  /**
   * @brief Publish the image that is already captured by capture().
   *
   */
  void publish();

  /**
   * @brief accessor of CameraInfo.
   *
   * you have to call capture() before call this.
   *
   * @return CameraInfo
   */
  inline const sensor_msgs::msg::CameraInfo &getInfo() const
  {
    return info_;
  }

  /**
   * @brief accessor of cv::Mat
   *
   * you have to call capture() before call this.
   *
   * @return captured cv::Mat
   */
  inline const cv::Mat &getCvImage() const
  {
    return bridge_.image;
  }

  /**
   * @brief accessor of ROS Image message.
   *
   * you have to call capture() before call this.
   *
   * @return message pointer.
   */
  inline const sensor_msgs::msg::Image::SharedPtr getImageMsgPtr() const
  {
    return bridge_.toImageMsg();
  }

  /**
   * @brief accessor of ROS Image viz message.
   *
   * you have to call capture() before call this.
   *
   * @return message pointer.
   */
  inline const sensor_msgs::msg::Image::SharedPtr getImageVizMsgPtr() const
  {
    return bridge_viz_.toImageMsg();
  }

  /**
   * @brief accessor of ROS Image calibration message.
   *
   * you have to call capture() before call this.
   *
   * @return message pointer.
   */
  inline const sensor_msgs::msg::Image::SharedPtr getImageCalMsgPtr() const
  {
    return bridge_cal_.toImageMsg();
  }

  /**
   * @brief try capture image width
   * @return true if success
   */
  inline bool setWidth(int32_t width)
  {
    return cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
  }

  /**
   * @brief try capture image height
   * @return true if success
   */
  inline bool setHeight(int32_t height)
  {
    return cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  }

  /**
   * @brief set cv::PROP_*
   * @return true if success
   */
  bool setPropertyFromParam(int property_id, const std::string &param_name);

  bool setY16();

  void vizClickCallback(const geometry_msgs::msg::Point& pt);

private:
  /**
   * @brief rescale camera calibration to another resolution
   */
  void rescaleCameraInfo(uint width, uint height);

  /**
   * @brief node handle for advertise.
   */
  rclcpp::Node::SharedPtr node_;

  /**
   * @brief ROS image transport utility.
   */
  image_transport::ImageTransport it_;

  /**
   * @brief name of topic without namespace (usually "image_raw").
   */
  std::string topic_name_;

  /**
   * @brief header.frame_id for publishing images.
   */
  std::string frame_id_;
  /**
   * @brief size of publisher buffer
   */
  uint32_t buffer_size_;

  /**
   * @brief image publisher created by image_transport::ImageTransport.
   */
  image_transport::CameraPublisher pub_;

  /**
   * @brief capture device.
   */
  cv::VideoCapture cap_;

  /**
   * @brief this stores last captured image.
   */
  cv_bridge::CvImage bridge_;

  /**
   * @brief this stores last captured image info.
   *
   * currently this has image size (width/height) only.
   */
  sensor_msgs::msg::CameraInfo info_;

  /**
   * @brief camera info manager
   */
  camera_info_manager::CameraInfoManager info_manager_;

  /**
   * @brief image publisher for visualization
   */
  image_transport::CameraPublisher pub_viz_;

  /**
   * @brief image publisher for visualization
   */
  image_transport::CameraPublisher pub_cal_;

  /**
   * @brief this stores last visualization image
   */
  cv_bridge::CvImage bridge_viz_;

  /**
   * @brief this stores last calibration image
   */
  cv_bridge::CvImage bridge_cal_;

  /**
   * @brief this stores info about visualization image
   *
   * currently this has image size (width/height) only.
   */
  sensor_msgs::msg::CameraInfo info_viz_;

  /** @brief this stores info about the calibration image
   *
   * currently this has image size (width/height) only.
   */
  sensor_msgs::msg::CameraInfo info_cal_;

  std::string clicktopic = topic_name_+"_viz_mouse_left";

  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pubmax;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pubmin;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr pickedpoint;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr clicksub;

  /**
   * @brief rescale_camera_info param value
   */
  bool rescale_camera_info_;

  /**
   * @brief capture_delay param value
   */
  rclcpp::Duration capture_delay_;

  /**
   * @brief mirror_horizontal_ param value
   */
  bool mirror_horizontal_;

    /**
   * @brief mirror_vertical_ param value
   */
  bool mirror_vertical_;

  /**
   * @brief publish_viz_ param value
   */
  bool publish_viz_;

  /**
   * @brief publish_cal_ param value
   */
  bool publish_cal_;

  /**
   * @brief invert_cal_ param value
   */
  bool invert_cal_;

  int ptx = 0; 
  int pty = 0;
};

} // namespace cv_camera

#endif // CV_CAMERA_CAPTURE_H
