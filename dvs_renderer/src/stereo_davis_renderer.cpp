#include "dvs_renderer/stereo_davis_renderer.h"
#include <std_msgs/Float32.h>

namespace dvs_renderer
{
StereoDavisRenderer::StereoDavisRenderer(ros::NodeHandle& nh, ros::NodeHandle nh_private)
  : nh_(nh), nhp_(nh_private)  //, image_tracking_(nh)
{
  //  got_camera_info_ = false;
  got_camera_info_left_ = false;
  got_camera_info_right_ = false;
  got_stereo_camera_model_ = false;

  // get parameters of display method
  std::string display_method_str;
  nhp_.param<std::string>("display_method", display_method_str, "grayscale");
  display_method_ = (display_method_str == std::string("grayscale")) ? GRAYSCALE : RED_BLUE;
  ROS_INFO("display_method: %s", display_method_str.c_str());

  integration_length_ = static_cast<size_t>(nh_private.param("integration_length", 2500));
  use_milliseconds_ = nh_private.param("use_milliseconds", false);
  ROS_INFO("integration_length: %lu", integration_length_);

  // setup subscribers and publishers
  event_left_sub_ = nh_.subscribe("/events_left", 1, &StereoDavisRenderer::eventsLeftCallback, this);
  event_right_sub_ = nh_.subscribe("/events_right", 1, &StereoDavisRenderer::eventsRightCallback, this);
  // events_left > /davis_left/events

  camera_info_left_sub_ = nh_.subscribe("/camera_info_left", 1, &StereoDavisRenderer::cameraInfoLeftCallback, this);
  camera_info_right_sub_ = nh_.subscribe("/camera_info_right", 1, &StereoDavisRenderer::cameraInfoRightCallback, this);
  // /davis_left/camera_info

  image_transport::ImageTransport it_(nh_);
  image_left_sub_ = it_.subscribe("/image_left", 1, &StereoDavisRenderer::imageLeftCallback, this);
  // /davis_left/image_raw
  image_right_sub_ = it_.subscribe("/image_right", 1, &StereoDavisRenderer::imageRightCallback, this);

  image_left_pub_ = it_.advertise("/dvs_rendering_left", 1);
  //  // /davis_left/dvs_rendering
  image_right_pub_ = it_.advertise("/dvs_rendering_right", 1);

  undistorted_image_left_pub_ = it_.advertise("/dvs_rectified_left", 1);
  //  // /davis_left/dvs_rectified
  undistorted_image_right_pub_ = it_.advertise("/dvs_rectified_right", 1);

  //  for (int i = 0; i < 2; ++i)
  //    for (int k = 0; k < 2; ++k)
  //      event_stats_[i].events_counter_[k] = 0;
  //  event_stats_[0].dt = 1;
  //  event_stats_[0].events_mean_lasttime_ = 0;
  //  event_stats_[0].events_mean_[0] = nh_.advertise<std_msgs::Float32>("events_on_mean_1", 1);
  //  event_stats_[0].events_mean_[1] = nh_.advertise<std_msgs::Float32>("events_off_mean_1", 1);
  //  event_stats_[1].dt = 5;
  //  event_stats_[1].events_mean_lasttime_ = 0;
  //  event_stats_[1].events_mean_[0] = nh_.advertise<std_msgs::Float32>("events_on_mean_5", 1);
  //  event_stats_[1].events_mean_[1] = nh_.advertise<std_msgs::Float32>("events_off_mean_5", 1);
}

StereoDavisRenderer::~StereoDavisRenderer()
{
  image_left_pub_.shutdown();
  image_right_pub_.shutdown();
  undistorted_image_left_pub_.shutdown();
  undistorted_image_right_pub_.shutdown();
}

void StereoDavisRenderer::cameraInfoLeftCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  got_camera_info_left_ = true;
  camera_info_left_ = *msg;

  if (got_camera_info_left_ && got_camera_info_right_)
  {
    if (!got_stereo_camera_model_)
    {
      got_stereo_camera_model_ = true;
      stereo_camera_model_.fromCameraInfo(camera_info_left_, camera_info_right_);
      ROS_INFO("stereo camera model done");
    }
  }
}

void StereoDavisRenderer::cameraInfoRightCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  got_camera_info_right_ = true;
  camera_info_right_ = *msg;

  if (got_camera_info_left_ && got_camera_info_right_)
  {
    if (!got_stereo_camera_model_)
    {
      got_stereo_camera_model_ = true;
      stereo_camera_model_.fromCameraInfo(camera_info_left_, camera_info_right_);
      ROS_INFO("stereo camera model done");
    }
  }
}

// void StereoDavisRenderer::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
//{
//  got_camera_info_ = true;

//  camera_matrix_ = cv::Mat(3, 3, CV_64F);
//  for (int i = 0; i < 3; i++)
//    for (int j = 0; j < 3; j++)
//      camera_matrix_.at<double>(cv::Point(i, j)) = msg->K[i + j * 3];

//  dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
//  for (int i = 0; i < msg->D.size(); i++)
//    dist_coeffs_.at<double>(i) = msg->D[i];
//}

void StereoDavisRenderer::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  //  image_tracking_.imageCallback(msg);

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // convert from grayscale to color image
  cv::cvtColor(cv_ptr->image, last_image_, CV_GRAY2BGR);
}

void StereoDavisRenderer::imageLeftCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  //  image_tracking_.imageCallback(msg);

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // convert from grayscale to color image
  cv::cvtColor(cv_ptr->image, last_image_left_, CV_GRAY2BGR);
  //  ROS_INFO("left image done");
}

void StereoDavisRenderer::imageRightCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  //  image_tracking_.imageCallback(msg);

  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // convert from grayscale to color image
  cv::cvtColor(cv_ptr->image, last_image_right_, CV_GRAY2BGR);
  //  ROS_INFO("right image done");
}

void StereoDavisRenderer::eventsLeftCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  for (const auto& e : msg->events)
  {
    events_left_.push_back(e);

    if (events_right_.size() >= integration_length_)
    {
      publishImageAndClearEvents();
    }
  }
}

void StereoDavisRenderer::eventsRightCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  for (const auto& e : msg->events)
  {
    events_right_.push_back(e);
  }
}

void StereoDavisRenderer::publishImageAndClearEvents()
{
  if (!last_image_left_.data || !last_image_right_.data)
    return;

  // only create image if at least one subscriber
  if (image_left_pub_.getNumSubscribers() > 0 && image_right_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image_left;
    cv_bridge::CvImage cv_image_right;

    if (display_method_ == RED_BLUE)
    {
      //      ROS_INFO("red blue start");
      cv_image_left.encoding = "bgr8";
      cv_image_right.encoding = "bgr8";

      //      ROS_INFO("copy last image");
      last_image_left_.copyTo(cv_image_left.image);
      last_image_right_.copyTo(cv_image_right.image);
      //      ROS_INFO("done");

      // set time stamp as the time in the middle of the event time interval
      //      ROS_INFO("set time stamp");
      ros::Time time_stamp;
      time_stamp.fromNSec(static_cast<uint64_t>(events_left_.front().ts.toNSec() +
                                                (events_left_.back().ts - events_left_.front().ts).toNSec() / 2.0));
      cv_image_left.header.stamp = time_stamp;
      cv_image_right.header.stamp = time_stamp;
      //      ROS_INFO("done");

      //      ROS_INFO("set event left");
      // left image
      for (size_t i = 0; i < events_left_.size(); ++i)
      {
        const int x = events_left_[i].x;
        const int y = events_left_[i].y;

        cv_image_left.image.at<cv::Vec3b>(cv::Point(x, y)) =
            (events_left_[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }
      //      ROS_INFO("done");

      //      ROS_INFO("set event right");
      // right image
      for (size_t i = 0; i < events_right_.size(); ++i)
      {
        const int x = events_right_[i].x;
        const int y = events_right_[i].y;

        cv_image_right.image.at<cv::Vec3b>(cv::Point(x, y)) =
            (events_right_[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }

      //      for (int i = 0; i < events_.size(); ++i)
      //      {
      //        const int x = events_[i].x;
      //        const int y = events_[i].y;

      //        cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) =
      //            (events_[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      //      }
      //      ROS_INFO("done");
    }

    else
    {
      cv_image_left.encoding = "mono8";
      cv_image_left.image = cv::Mat(last_image_left_.size(), CV_8U);
      cv_image_left.image = cv::Scalar(128);

      cv_image_right.encoding = "mono8";
      cv_image_right.image = cv::Mat(last_image_right_.size(), CV_8U);
      cv_image_right.image = cv::Scalar(128);

      cv::Mat on_events_left = cv::Mat(last_image_left_.size(), CV_8U);
      on_events_left = cv::Scalar(0);
      cv::Mat off_events_left = cv::Mat(last_image_left_.size(), CV_8U);
      off_events_left = cv::Scalar(0);

      cv::Mat on_events_right = cv::Mat(last_image_right_.size(), CV_8U);
      on_events_right = cv::Scalar(0);
      cv::Mat off_events_right = cv::Mat(last_image_right_.size(), CV_8U);
      off_events_right = cv::Scalar(0);

      // left image
      for (size_t i = 0; i < events_left_.size(); ++i)
      {
        const int x = events_left_[i].x;
        const int y = events_left_[i].y;

        if (events_left_[i].polarity == 1)
          on_events_left.at<uint8_t>(cv::Point(x, y))++;
        else
          off_events_left.at<uint8_t>(cv::Point(x, y))++;
      }

      // right image
      for (size_t i = 0; i < events_right_.size(); ++i)
      {
        const int x = events_right_[i].x;
        const int y = events_right_[i].y;

        if (events_right_[i].polarity == 1)
          on_events_right.at<uint8_t>(cv::Point(x, y))++;
        else
          off_events_right.at<uint8_t>(cv::Point(x, y))++;
      }

      // count events per pixels with polarity
      //      for (int i = 0; i < events_.size(); ++i)
      //      {
      //        const int x = events_[i].x;
      //        const int y = events_[i].y;

      //        if (events_[i].polarity == 1)
      //          on_events.at<uint8_t>(cv::Point(x, y))++;
      //        else
      //          off_events.at<uint8_t>(cv::Point(x, y))++;
      //      }

      // scale image
      cv::normalize(on_events_left, on_events_left, 0, 128, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(off_events_left, off_events_left, 0, 127, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(on_events_right, on_events_right, 0, 128, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(off_events_right, off_events_right, 0, 127, cv::NORM_MINMAX, CV_8UC1);

      cv_image_left.image += on_events_left;
      cv_image_left.image -= off_events_left;
      cv_image_right.image += on_events_right;
      cv_image_right.image -= off_events_right;
    }

    image_left_pub_.publish(cv_image_left.toImageMsg());
    image_right_pub_.publish(cv_image_right.toImageMsg());

    if (got_stereo_camera_model_ && undistorted_image_left_pub_.getNumSubscribers() > 0 &&
        undistorted_image_right_pub_.getNumSubscribers() > 0)
    {
      ROS_INFO("rectify start");
      cv_bridge::CvImage cv_image2_left;
      cv_bridge::CvImage cv_image2_right;
      cv_image2_left.encoding = cv_image_left.encoding;
      cv_image2_right.encoding = cv_image_right.encoding;

      stereo_camera_model_.left().rectifyImage(cv_image_left.image, cv_image2_left.image);
      stereo_camera_model_.right().rectifyImage(cv_image_right.image, cv_image2_right.image);

      //      cv::undistort(cv_image.image, cv_image2.image, camera_matrix_, dist_coeffs_);

      undistorted_image_left_pub_.publish(cv_image2_left.toImageMsg());
      undistorted_image_right_pub_.publish(cv_image2_right.toImageMsg());
    }
  }

  events_left_.clear();
  events_right_.clear();
}

void StereoDavisRenderer::setStereoCameraModel()
{
  got_camera_info_ = true;

  // set camera names and load calibration data
  ros::NodeHandle nh_cam_left = ros::NodeHandle(nhp_, "davis_left");
  ros::NodeHandle nh_cam_right = ros::NodeHandle(nhp_, "davis_right");

  std::string cam_name_left;
  std::string cam_name_right;

  nh_cam_left.param<std::string>("camera_name", cam_name_left, "DAVIS-84010032");
  nh_cam_right.param<std::string>("camera_name", cam_name_right, "DAVIS-84010059");

  ROS_INFO("Left Camera Name : [%s]", cam_name_left.c_str());
  ROS_INFO("Right Camera Name: [%s]", cam_name_right.c_str());

  camera_info_manager::CameraInfoManager camera_info_left(nh_cam_left, cam_name_left);
  camera_info_manager::CameraInfoManager camera_info_right(nh_cam_right, cam_name_right);
  stereo_camera_model_.fromCameraInfo(camera_info_left.getCameraInfo(), camera_info_right.getCameraInfo());
}

void StereoDavisRenderer::publishStats()
{
  std_msgs::Float32 msg;
  ros::Time now = ros::Time::now();
  for (int i = 0; i < 2; ++i)
  {
    if (event_stats_[i].events_mean_lasttime_ + event_stats_[i].dt <= now.toSec())
    {
      event_stats_[i].events_mean_lasttime_ = now.toSec();
      for (int k = 0; k < 2; ++k)
      {
        msg.data = static_cast<float>(event_stats_[i].events_counter_[k] / event_stats_[i].dt);
        event_stats_[i].events_mean_[k].publish(msg);
        event_stats_[i].events_counter_[k] = 0;
      }
    }
  }
}

}  // namespace
