#ifndef IMGPREPNODE_HPP
#define IMGPREPNODE_HPP

/// Some parts of this package were modified from:
/// https://github.com/omwdunkley/ollieRosTools.git

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <tud_img_prep/dynamic_param_configConfig.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <map>



class ImgPrepNode {
 public:
  ImgPrepNode();

  ros::NodeHandle nh;
  // Function Callbacks
  void imageCb(const sensor_msgs::ImageConstPtr &input_image_msg);
  void cameraInfoCb(const sensor_msgs::CameraInfo &cam_info);
  void dynamicReconfigureCb(
      tud_img_prep::dynamic_param_configConfig& config, uint32_t level);

  //image processing functions
  void recomputeLUT(const float brightness, const float contrast);
  cv::Mat deinterlaceCut(const cv::Mat& in, const bool even = false) const;
  cv::Mat deinterlace(const cv::Mat& in, const int interpolation) const;
  cv::Mat process(const cv::Mat& in) const;

private:
  // dynamic reconfigure server
  dynamic_reconfigure::Server<tud_img_prep::dynamic_param_configConfig>
      server_;
  ros::Publisher cam_info_pub_;
  image_transport::CameraPublisher img_prep_pub_;

  ros::Subscriber cam_info_sub_;
  image_transport::Subscriber cam_img_sub_;

  image_transport::ImageTransport m_image_transport;

  sensor_msgs::CameraInfo cam_info_msg_;
  camera_info_manager::CameraInfoManager *cinfo_;



  cv_bridge::CvImageConstPtr cv_ptr_;

  // Parameters (dynamic reconfigure ROS)
  int color_encoding_;
  std::map<int, std::string> colors_;
  int deinterlace_method_;
  int equalization_method_;
  bool equalize_color_;
  bool node_on_;
  bool filter_median_;
  bool filter_gaussian_;
  bool filter_bilateral_;


  int k_median_;
  int k_gaussian_;
  int k_bilateral_;
  double sigmaX_gaussian_;
  double sigmaY_gaussian_;
  double sigmaX_bilateral_;
  double sigmaY_bilateral_;
  double brightness_;
  double contrast_;

  cv::Mat lut_;

  //Maybe include
  int treshold_;


  float timeAlpha_;
  float timeAvg_;
};

#endif  // IMGPREPNODE_HPP
