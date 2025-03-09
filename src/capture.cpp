// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/capture.h"
#include <sstream>
#include <string>
#include <regex>

namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;
using namespace std::literals;

Capture::Capture(rclcpp::Node::SharedPtr node, const std::string& topic_name, uint32_t buffer_size,
                 const std::string& frame_id)
  : node_(node)
  , it_(node_)
  , topic_name_(topic_name)
  , frame_id_(frame_id)
  , buffer_size_(buffer_size)
  , info_manager_(node_.get(), frame_id)
  , capture_delay_(rclcpp::Duration(rclcpp::Duration::from_seconds(node_->get_parameter_or("capture_delay", 0.0))))
  , mirror_horizontal_(node_->get_parameter_or("mirror_horizontal", false))
  , mirror_vertical_(node_->get_parameter_or("mirror_vertical", false))
  , publish_viz_(node_->get_parameter_or("pub_vizualization", true))
  , publish_cal_(node_->get_parameter_or("pub_calibration", false))
  , invert_cal_(node_->get_parameter_or("invert_calibration", false))
{
}

void Capture::loadCameraInfo()
{
  std::string url;
  if (node_->get_parameter("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }

  rescale_camera_info_  = false;
  node_->get_parameter_or("rescale_camera_info", rescale_camera_info_, rescale_camera_info_);

  for (int i = 0;; ++i)
  {
    int code = 0;
    double value = 0.0;
    std::stringstream stream;
    stream << "property_" << i << "_code";
    const std::string param_for_code = stream.str();
    stream.str("");
    stream << "property_" << i << "_value";
    const std::string param_for_value = stream.str();
    if (!node_->get_parameter(param_for_code, code) || !node_->get_parameter(param_for_value, value))
    {
      break;
    }
    if (!cap_.set(code, value))
    {
      RCLCPP_ERROR(node_->get_logger(),"Setting with code %d and value %f failed", code, value);
    }
  }
}

void Capture::rescaleCameraInfo(uint width, uint height)
{
  double width_coeff = static_cast<double>(width) / info_.width;
  double height_coeff = static_cast<double>(height) / info_.height;
  info_.width = width;
  info_.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  info_.k[0] *= width_coeff;
  info_.k[2] *= width_coeff;
  info_.k[4] *= height_coeff;
  info_.k[5] *= height_coeff;

  info_.p[0] *= width_coeff;
  info_.p[2] *= width_coeff;
  info_.p[5] *= height_coeff;
  info_.p[6] *= height_coeff;
}

void Capture::open(int32_t device_id)
{
  cap_.open(device_id,cv::CAP_V4L2);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "device_id" << device_id << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  if (publish_viz_){pub_viz_ = it_.advertiseCamera(topic_name_+"_viz", buffer_size_);}
  if (publish_cal_){pub_cal_ = it_.advertiseCamera(topic_name_+"_cal", buffer_size_);}

  loadCameraInfo();
}

void Capture::open(const std::string &device_sn)
{
  // Search USB devices for matching serial number using udevadm output
  std::array<char, 128> buffer;
  int device_open_id;
  std::string result, current_line, device_num;

  for(int i = 0; i < 30; i++){
  std::string cmdstr = std::string("udevadm info --name=/dev/video") + std::to_string(i);
  const char* cmd = cmdstr.c_str();
  std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
  if (!pipe) throw std::runtime_error("popen() failed!");
  while (!feof(pipe.get())) {
      if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
          result += buffer.data();
  }
  std::istringstream lsusboutut{result};

  while (std::getline(lsusboutut, current_line))
  {
    std::smatch m; 
    if(std::regex_search(current_line, m, std::regex("N: video(\\d+)"))){
      device_num = m[1].str();
    }

    std::smatch mm; 
    if(std::regex_search(current_line, mm, std::regex(device_sn.c_str()))){
      device_open_id = std::stoi(device_num);
      break;
    }
  }
  // TODO: this doesn't check if multiple devices match the serial, just uses the last
  }
  RCLCPP_INFO(node_->get_logger(),"Found device with matching serial number: /dev/video%s, %s\n", device_num.c_str(), device_sn.c_str());

  cap_.open(device_open_id, cv::CAP_V4L2);
  if (!cap_.isOpened())
  {
    throw DeviceError("device_sn " + device_sn + " cannot be opened");
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  if (publish_viz_){pub_viz_ = it_.advertiseCamera(topic_name_+"_viz", buffer_size_);}
  if (publish_cal_){pub_cal_ = it_.advertiseCamera(topic_name_+"_cal", buffer_size_);}

  loadCameraInfo();
}

void Capture::open()
{
  open(0);
}

void Capture::openFile(const std::string &file_path)
{
  cap_.open(file_path);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "file " << file_path << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  if (publish_viz_){pub_viz_ = it_.advertiseCamera(topic_name_+"_viz", buffer_size_);}
  if (publish_cal_){pub_cal_ = it_.advertiseCamera(topic_name_+"_cal", buffer_size_);}

  loadCameraInfo();
}

bool Capture::capture()
{
  if (cap_.read(bridge_.image))
  {
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    rclcpp::Time stamp = system_clock.now() - capture_delay_;

    // Trim off lepton metadata
    // int cropheight = bridge_.image.size().height - 2;
    // int cropwidth = bridge_.image.size().width;
    // bridge_.image = bridge_.image(cv::Range(0,cropheight),cv::Range(0,cropwidth));

    // Mirror
    if (mirror_horizontal_)
    {
      cv_bridge::CvImage tmp_;
      cv::flip(bridge_.image, tmp_.image, 1);
      bridge_.image = tmp_.image;
    }

    if (mirror_vertical_)
    {
      cv_bridge::CvImage tmp_;
      cv::flip(bridge_.image, tmp_.image, 0);
      bridge_.image = tmp_.image;
    }

    // Construct header
    bridge_.encoding = enc::MONO16;
    bridge_.header.stamp = stamp;
    bridge_.header.frame_id = frame_id_;

    info_ = info_manager_.getCameraInfo();
    if (info_.height == 0 && info_.width == 0)
    {
      info_.height = bridge_.image.rows;
      info_.width = bridge_.image.cols;
    }
    else if (info_.height != bridge_.image.rows || info_.width != bridge_.image.cols)
    {
      if (rescale_camera_info_)
      {
        int old_width = info_.width;
        int old_height = info_.height;
        rescaleCameraInfo(bridge_.image.cols, bridge_.image.rows);
        RCLCPP_INFO_ONCE(node_->get_logger(),"Camera calibration automatically rescaled from %dx%d to %dx%d",
                      old_width, old_height, bridge_.image.cols, bridge_.image.rows);
      }
      else
      {
        RCLCPP_WARN_ONCE(node_->get_logger(),"Calibration resolution %dx%d does not match camera resolution %dx%d. "
                      "Use rescale_camera_info param for rescaling",
                      info_.width, info_.height, bridge_.image.cols, bridge_.image.rows);
      }
    }
    info_.header.stamp = stamp;
    info_.header.frame_id = frame_id_;

    // Get high & low IR intensities
    double minVal; 
    double maxVal; 
    cv::Point minLoc; 
    cv::Point maxLoc;

    minMaxLoc( bridge_.image, &minVal, &maxVal, &minLoc, &maxLoc );
    example_interfaces::msg::Float64 maxmsg;
    maxmsg.data = maxVal;
    this->pubmax->publish(maxmsg);

    example_interfaces::msg::Float64 minmsg;
    minmsg.data = minVal;
    this->pubmin->publish(minmsg);

    if (publish_viz_)
    {
      // Make a pretty image
      cv_bridge::CvImage tmp_;
      cv::normalize(bridge_.image, tmp_.image, 0, 65535, cv::NORM_MINMAX);
      tmp_.image.convertTo(tmp_.image,CV_8UC1,1/255.0); //CV_32F
      cv::applyColorMap(tmp_.image, bridge_viz_.image, cv::COLORMAP_HOT);

      bridge_viz_.encoding = enc::BGR8;
      bridge_viz_.header.stamp = stamp;
      bridge_viz_.header.frame_id = frame_id_;

      info_viz_ = info_;

      // Publish temperature at picked point
      uint16_t temp_mk = bridge_.image.at<uint16_t>(pty,ptx);
      example_interfaces::msg::Float64 intensitymsg;
      intensitymsg.data = static_cast< float >( temp_mk );
      this->pickedpoint->publish(intensitymsg);
      
      // sensor_msgs::Temperature tempmsg;
      // tempmsg.temperature = static_cast< float >( temp_mk ) / 100 - 271.15;
      // tempmsg.header.stamp = stamp;
      // pointtemp.publish(tempmsg);

      // Draw crosshair
      cv::line(bridge_viz_.image, cv::Point(ptx - 5, pty), cv::Point(ptx - 2, pty), cvScalar(117,79,142), 1);
      cv::line(bridge_viz_.image, cv::Point(ptx + 5, pty), cv::Point(ptx + 2, pty), cvScalar(117,79,142), 1);
      cv::line(bridge_viz_.image, cv::Point(ptx, pty - 5), cv::Point(ptx, pty - 2), cvScalar(117,79,142), 1);
      cv::line(bridge_viz_.image, cv::Point(ptx, pty + 5), cv::Point(ptx, pty + 2), cvScalar(117,79,142), 1);

    }

    if (publish_cal_)
    {
      // Make an image useful for calibrating the camera
      cv_bridge::CvImage tmpcal_;
      cv::normalize(bridge_.image, bridge_cal_.image, 0, 65535, cv::NORM_MINMAX);
      bridge_cal_.image.convertTo(bridge_cal_.image,CV_8UC1,1/255.0); 
      if(invert_cal_){
        cv::bitwise_not(bridge_cal_.image, bridge_cal_.image);
      }
      // cv::equalizeHist( bridge_cal_.image, bridge_cal_.image );
      cv::GaussianBlur( bridge_cal_.image, bridge_cal_.image, cv::Size( 3, 3 ), 0, 0 );

      bridge_cal_.encoding = enc::MONO8;
      bridge_cal_.header.stamp = stamp;
      bridge_cal_.header.frame_id = frame_id_;

      info_cal_ = info_;
    }
    return true;
  }
  return false;
}

void Capture::publish()
{
  pub_.publish(*getImageMsgPtr(), info_);
  if(publish_viz_){
    pub_viz_.publish(*getImageVizMsgPtr(), info_viz_);
  }
  if(publish_cal_){
    pub_cal_.publish(*getImageCalMsgPtr(), info_cal_);
  }
}

bool Capture::setPropertyFromParam(int property_id, const std::string &param_name)
{
  if (cap_.isOpened())
  {
    double value = 0.0;
    if (node_->get_parameter(param_name, value))
    {
      RCLCPP_INFO(node_->get_logger(),"setting property %s = %lf", param_name.c_str(), value);
      return cap_.set(property_id, value);
    }
  }
  return true;
}

bool Capture::setY16()
{
  if (cap_.isOpened())
  {
    double yval = cv::VideoWriter::fourcc('Y','1','6',' ');
    RCLCPP_INFO(node_->get_logger(),"setting property Y16 = %lf", yval);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','1','6',' '));
  }
  return true;
}

void Capture::vizClickCallback(const geometry_msgs::msg::Point& pt)
{
  RCLCPP_INFO(node_->get_logger(),"Monitoring point temperature at: (%f,%f)",pt.x,pt.y);
  ptx = static_cast<int>(pt.x);
  pty = static_cast<int>(pt.y);
}

} // namespace cv_camera
