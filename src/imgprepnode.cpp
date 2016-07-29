#include "tud_img_prep/imgprepnode.hpp"
#include <random>

ImgPrepNode::ImgPrepNode()
    : m_image_transport(nh),
      node_on_(true),
      color_encoding_(2),
      deinterlace_method_(false),
      equalization_method_(false),
      equalize_color_(false),
      filter_median_(true),
      filter_gaussian_(false),
      filter_bilateral_(false),
      k_median_(0),
      k_gaussian_(0),
      k_bilateral_(0),
      sigmaX_gaussian_(1),
      sigmaY_gaussian_(1),
      sigmaX_bilateral_(1),
      sigmaY_bilateral_(1),
      brightness_(0.0),
      contrast_(0.0),
      timeAlpha_(0.95),
      timeAvg_(0.0)
{
  ros::NodeHandle params("~");
  // Topic Parameters

  std::string camera_namespace;

  params.param<std::string>("camera_namespace", camera_namespace, "/cam");

  cam_img_sub_ = m_image_transport.subscribe(camera_namespace + "/image_raw", 1,
                                             &ImgPrepNode::imageCb, this);
  cam_info_sub_ = nh.subscribe(camera_namespace + "/camera_info", 1,
                               &ImgPrepNode::cameraInfoCb, this);
  img_prep_pub_ = m_image_transport.advertiseCamera(
      "prep/" + camera_namespace + "/image_raw", 1);

  // Dynamic parameter reconfigure
  dynamic_reconfigure::Server<
      tud_img_prep::dynamic_param_configConfig>::CallbackType f;
  f = boost::bind(&ImgPrepNode::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
  recomputeLUT(0.0, 0.0);
  colors_[0] = ""; // passthrough
  colors_[1] = sensor_msgs::image_encodings::BGR8;
  colors_[2] = sensor_msgs::image_encodings::RGB8;
  colors_[3] = sensor_msgs::image_encodings::MONO8;
}

void ImgPrepNode::imageCb(const sensor_msgs::ImageConstPtr& input_image_msg)
{
  if (img_prep_pub_.getNumSubscribers() > 0)
  {
    /// Measure HZ, Processing time, Image acquisation time
    ros::WallTime time_s0 = ros::WallTime::now();

    cv_ptr_ = cv_bridge::toCvShare(input_image_msg, colors_[color_encoding_]);
    cv::Mat image = process(cv_ptr_->image);

    // OpenCV to MAT structure
    // m_I = cv_ptr_->image;

    // Gaussian Blur
    // cv::GaussianBlur(m_I, m_I_filtered, cv::Size(3,3), 0, 0,
    // cv::BORDER_DEFAULT);

    // Weights
    // cv::addWeighted(I, 2.5, I_filtered, -1.5, 0, I_filtered);

    /// Convert the image to grayscale
    // cv::cvtColor( m_I_filtered, m_I_filtered, CV_RGB2GRAY );

    // Equalize histogram
    // cv::equalizeHist(m_I_filtered,m_I_filtered);

    // Treshold
    // cv::threshold(m_I_filtered,m_I_filtered,treshold_,0,3);

    // cv::cvtColor(m_I_filtered,m_I_filtered,CV_GRAY2RGB);

    // Creating new filtered image_raw and publishing
    // cv_bridge::CvImagePtr in_msg;
    // in_msg = cv_ptr_;
    cv_bridge::CvImage out_msg;

    // Same timestamp and tf frame as input image
    // out_msg.header = cv_ptr_->header;
    out_msg.header = cv_ptr_->header;

    // Format
    // out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    // out_msg.image = m_I_filtered;
    out_msg.encoding = colors_[color_encoding_];
    out_msg.image = image;

    // To publish same camera calibration information as input image
    // cam_info_msg_.header.stamp = out_msg.header.stamp;
    // cam_info_msg_.header.frame_id = "/ardrone/bottom/filtered";
    cam_info_msg_.header.stamp = out_msg.header.stamp;
    cam_info_msg_.header.frame_id = out_msg.header.frame_id;
    cam_info_msg_.header.seq = out_msg.header.seq;

    // Publish filtered image
    // img_prep_pub_.publish(*out_msg.toImageMsg(),cam_info_msg_);
    img_prep_pub_.publish(*out_msg.toImageMsg(), cam_info_msg_);

    // Compute running average of processing time
    timeAvg_ = timeAvg_ * timeAlpha_ +
               (1.0 - timeAlpha_) * (ros::WallTime::now() - time_s0).toSec();
    ROS_DEBUG_THROTTLE(1, "Processing TimeXYZ: %.1fms", timeAvg_ * 1000.);
  }
  else
  {
    // Nothing
  }
}

void ImgPrepNode::cameraInfoCb(const sensor_msgs::CameraInfo& cam_info)
{
  // Receive input image calibration information
  cam_info_msg_ = cam_info;
}

void ImgPrepNode::dynamicReconfigureCb(
    tud_img_prep::dynamic_param_configConfig& config, uint32_t level)
{
  ROS_DEBUG("Reconfigure Request");
  /// Turn node on and off when settings change
  if (node_on_ != config.node_on)
  {
    node_on_ = config.node_on;
    if (node_on_)
    {
      // Node was just turned on
      // subImage = imTransport.subscribe(inputTopic, 1,
      // &PreProcNode::incomingImage, this);
      ROS_INFO("Node On");
    }
    else
    {
      // Node was just turned off
      // subImage.shutdown();
      ROS_INFO("Node Off");
    }
  }
  recomputeLUT(config.brightness, config.contrast);

  color_encoding_ = config.color_encoding;
  deinterlace_method_ = config.deinterlace_method;
  equalization_method_ = config.equalization_method;
  equalize_color_ = config.equalize_color;
  brightness_ = config.brightness;
  contrast_ = config.contrast;


  // kernel must be odd and greater than 1
  filter_median_ = config.filter_median;
  k_median_ = config.ksize_median * 2 + 1;

  filter_gaussian_ = config.filter_gaussian;
  k_gaussian_ = config.ksize_gaussian * 2 + 1;
  sigmaX_gaussian_ = config.sigmaX_gaussian;
  sigmaY_gaussian_ = config.sigmaY_gaussian;

  filter_bilateral_ = config.filter_bilateral;
  k_bilateral_ = config.ksize_bilateral * 2 + 1;
  sigmaX_bilateral_ = config.sigmaX_bilateral;
  sigmaY_bilateral_ = config.sigmaY_bilateral;

}

void ImgPrepNode::recomputeLUT(const float brightness, const float contrast)
{
  /// Recomputes the brightness/contrast look up table. Maps intensity values
  /// 0-255 to 0-255
  lut_ = cv::Mat(1, 256, CV_8U);

  if (contrast > 0)
  {
    const float delta = 127. * contrast;
    const float a = 255. / (255. - delta * 2);
    const float b = a * (brightness * 100 - delta);
    for (int i = 0; i < 256; ++i)
    {
      int v = round(a * i + b);
      if (v < 0)
      {
        v = 0;
      }
      if (v > 255)
      {
        v = 255;
      }
      lut_.at<uchar>(i) = static_cast<uchar>(v);
    }
  }
  else
  {
    const float delta = -128. * contrast;
    const float a = (256. - delta * 2) / 255.;
    const float b = a * brightness * 100. + delta;
    for (int i = 0; i < 256; ++i)
    {
      int v = round(a * i + b);
      if (v < 0)
      {
        v = 0;
      }
      if (v > 255)
      {
        v = 255;
      }
      lut_.at<uchar>(i) = static_cast<uchar>(v);
    }
  }
}

cv::Mat ImgPrepNode::deinterlaceCut(const cv::Mat& in, const bool even) const
{
  /// Return an image with either the even or the odd rows missing
  cv::Mat out = in.reshape(0, in.rows / 2);
  if (even)
  {
    out = out.colRange(0, out.cols / 2);
  }
  else
  {
    out = out.colRange(out.cols / 2, out.cols);
  }
  return out;
}

cv::Mat ImgPrepNode::deinterlace(const cv::Mat& in,
                                 const int interpolation) const
{
  cv::Mat half = deinterlaceCut(in);
  // make continuous again
  cv::Mat out(half.size(), half.type());
  half.copyTo(out);
  cv::resize(half, out, cv::Size(), 1, 2, interpolation);
  return out;
}

cv::Mat ImgPrepNode::process(const cv::Mat& in) const
{
  cv::Mat out;
  out = in.clone();
  /// Interlacing
  switch (deinterlace_method_)
  {
  case 0:
    // FULL_INTERLACED
    break;
  case 1:
    // HALF_CUT
    out = deinterlaceCut(in);
    break;
  case 2:
    // HALF_NEAREST
    out = deinterlace(in, cv::INTER_NEAREST);
    break;
  case 3:
    // HALF_LINEAR
    out = deinterlace(in, cv::INTER_LINEAR);
    break;
  case 4:
    // HALF_AREA
    out = deinterlace(in, cv::INTER_AREA);
    break;

  case 5:
    // HALF_CUBIC
    out = deinterlace(in, cv::INTER_CUBIC);
    break;

  case 6:
    // HALF_LANCZOS4
    out = deinterlace(in, cv::INTER_LANCZOS4);
    break;

  default: // Leave as is
    break;
  }

  /// Equalization (Set contrast and brightness)
  switch (equalization_method_)
  {
  case 0:
    // No equalization
    break;

  case 1:
    // Equalize using OpenCV Histogram functions
    if (out.channels() >= 3)
    {
      if (equalize_color_)
      {
        // Equalize each colour channel
        std::vector<cv::Mat> channels;
        split(out, channels);
        cv::equalizeHist(channels[0], channels[0]);
        cv::equalizeHist(channels[1], channels[1]);
        cv::equalizeHist(channels[2], channels[2]);
        cv::merge(channels, out);
      }
      else
      {
        // Convert to different color space and equalize the intensities only
        cv::Mat ycrcb;

        if (colors_.at(color_encoding_) == sensor_msgs::image_encodings::RGB8)
        {
          cv::cvtColor(out, ycrcb, CV_RGB2YCrCb);
        }
        else if (colors_.at(color_encoding_) == sensor_msgs::image_encodings::BGR8)
        {
          cv::cvtColor(out, ycrcb, CV_BGR2YCrCb);
        }

        std::vector<cv::Mat> channels;
        split(ycrcb, channels);
        cv::equalizeHist(channels[0], channels[0]);
        cv::merge(channels, ycrcb);

        if (colors_.at(color_encoding_) == sensor_msgs::image_encodings::RGB8)
        {
          cv::cvtColor(ycrcb, out, CV_YCrCb2RGB);
        }
        else if (colors_.at(color_encoding_) == sensor_msgs::image_encodings::BGR8)
        {
          cv::cvtColor(ycrcb, out, CV_YCrCb2BGR);
        }
      }
    }
    else
    {
      cv::equalizeHist(out, out);
    }
    break;
  case 2:
    // Manual equalization by using user defined contrast and brightness
    // values
    if (out.channels() >= 3)
    {
      if (equalize_color_)
      {
        // Equalize each colour channel
        cv::LUT(out, lut_, out);
      }
      else
      {
        // Convert to different color space and equalize the intensities only
        cv::Mat ycrcb;
        if (colors_.at(color_encoding_) == sensor_msgs::image_encodings::RGB8)
        {
          cv::cvtColor(out, ycrcb, CV_RGB2YCrCb);
        }
        else if (colors_.at(color_encoding_) == sensor_msgs::image_encodings::BGR8)
        {
          cv::cvtColor(out, ycrcb, CV_BGR2YCrCb);
        }

        std::vector<cv::Mat> channels;
        split(ycrcb, channels);
        cv::LUT(channels[0], lut_, channels[0]);
        cv::merge(channels, ycrcb);

        if (colors_.at(color_encoding_) == sensor_msgs::image_encodings::RGB8)
        {
          cv::cvtColor(ycrcb, out, CV_YCrCb2RGB);
        }
        else if (colors_.at(color_encoding_) == sensor_msgs::image_encodings::BGR8)
        {
          cv::cvtColor(ycrcb, out, CV_YCrCb2BGR);
        }
      }
    }
    else
    {
      cv::LUT(out, lut_, out);
    }
    break;
  default: // Leave as is
    break;
  }

  /// Smoothing/Filtering
  if (filter_median_)
  {
    cv::medianBlur(out, out, k_median_);
  }
  if (filter_gaussian_)
  {
    cv::GaussianBlur(out, out, cv::Size(k_gaussian_, k_gaussian_), sigmaX_gaussian_, sigmaY_gaussian_);
  }
  if (filter_bilateral_){
    cv::Mat dst;
    cv::bilateralFilter(out, dst, cv::max(1, static_cast<int>(floor(k_bilateral_ / 2.0))),
                        sigmaX_bilateral_, sigmaY_bilateral_);
    out = dst;
  }

  return out;
}
