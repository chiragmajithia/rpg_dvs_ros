#include "dvs_renderer/stereo_davis_renderer.h"
#include <std_msgs/Float32.h>
#include <iostream>
//#include <iterator>
#include <utility>
namespace dvs_renderer
{
StereoDavisRenderer::StereoDavisRenderer(ros::NodeHandle& nh, ros::NodeHandle nh_private)
  : nh_(nh), nhp_(nh_private)  //, image_tracking_(nh)
{
  got_camera_info_left_ = false;
  got_camera_info_right_ = false;
  got_stereo_camera_model_ = false;
  request_clear_left_queue_ = false;
  is_first_event_ = true;

  // get parameters of display method
  std::string display_method_str;
  nhp_.param<std::string>("display_method", display_method_str, "grayscale");
  display_method_ = (display_method_str == std::string("grayscale")) ? GRAYSCALE : RED_BLUE;
  ROS_INFO("display_method: %s", display_method_str.c_str());

  std::string rectification_method_str;
  nhp_.param<std::string>("rectification_method", rectification_method_str, "bilinear");
  rectification_method_ = (rectification_method_str == std::string("bilinear")) ? BILINEAR : NEAREST;

  integration_length_ = static_cast<size_t>(nhp_.param("integration_length", 2500));
  use_milliseconds_ = nh_private.param("use_milliseconds", false);
  ROS_INFO("integration_length: %lu", integration_length_);

  is_sync_ = nhp_.param("is_sync", false);

  // setup subscribers and publishers
  event_left_sub_ = nh_.subscribe("/events_left", 0, &StereoDavisRenderer::eventsLeftCallback, this);
  event_right_sub_ = nh_.subscribe("/events_right", 0, &StereoDavisRenderer::eventsRightCallback, this);

  camera_info_left_sub_ = nh_.subscribe("/camera_info_left", 1, &StereoDavisRenderer::cameraInfoLeftCallback, this);
  camera_info_right_sub_ = nh_.subscribe("/camera_info_right", 1, &StereoDavisRenderer::cameraInfoRightCallback, this);

  image_transport::ImageTransport it_(nh_);
  image_left_sub_ = it_.subscribe("/image_left", 1, &StereoDavisRenderer::imageLeftCallback, this);
  image_right_sub_ = it_.subscribe("/image_right", 1, &StereoDavisRenderer::imageRightCallback, this);

  image_left_pub_ = it_.advertise("/dvs_rendering_left", 1);
  image_right_pub_ = it_.advertise("/dvs_rendering_right", 1);

  rectified_image_left_pub_ = it_.advertise("/dvs_rectified_left", 1);
  rectified_image_right_pub_ = it_.advertise("/dvs_rectified_right", 1);
  rectified_image_stereo_pub_ = it_.advertise("/dvs_rectified_stereo", 1);

  reset_sub_ = nh_.subscribe("/reset_timestamps", 1, &StereoDavisRenderer::resetTimestampsCallback, this);

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
void StereoDavisRenderer::resetTimestampsCallback(const std_msgs::Time::ConstPtr& msg)
{
  ROS_INFO("stereo davis synchronized");
  is_sync_ = true;
}

StereoDavisRenderer::~StereoDavisRenderer()
{
  image_left_pub_.shutdown();
  image_right_pub_.shutdown();
  rectified_image_left_pub_.shutdown();
  rectified_image_right_pub_.shutdown();
}

void StereoDavisRenderer::imageLeftCallback(const sensor_msgs::Image::ConstPtr& msg)
{
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
}

void StereoDavisRenderer::imageRightCallback(const sensor_msgs::Image::ConstPtr& msg)
{
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
}

void StereoDavisRenderer::eventsLeftCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  if (is_sync_)
  {
    for (const auto& e : msg->events)
    {
      events_left_queue_.insert(std::make_pair(e.ts, e));
      if (events_left_queue_.size() >= integration_length_ && events_right_queue_.size() >= integration_length_)
      {
        publishImageAndClearEvents();
      }
    }
  }
}

void StereoDavisRenderer::eventsRightCallback(const dvs_msgs::EventArray::ConstPtr& msg)
{
  if (is_sync_)
  {
    for (const auto& e : msg->events)
    {
      events_right_queue_.insert(std::make_pair(e.ts, e));
      if (events_left_queue_.size() >= integration_length_ && events_right_queue_.size() >= integration_length_)
      {
        publishImageAndClearEvents();
      }
    }
  }
}

void StereoDavisRenderer::publishImageAndClearEvents()
{
  if (!last_image_left_.data || !last_image_right_.data)
  {
    return;
  }

  if ((image_left_pub_.getNumSubscribers() > 0 && image_right_pub_.getNumSubscribers() > 0) ||
      rectified_image_stereo_pub_.getNumSubscribers() > 0)
  {
    cv_bridge::CvImage cv_image_left;
    cv_bridge::CvImage cv_image_right;

    std::map<ros::Time, dvs_msgs::Event>::iterator it_left_begin = events_left_queue_.begin();
    std::map<ros::Time, dvs_msgs::Event>::iterator it_right_begin = events_right_queue_.begin();
    std::map<ros::Time, dvs_msgs::Event>::iterator it_left_end = events_left_queue_.end();
    std::map<ros::Time, dvs_msgs::Event>::iterator it_right_end = events_right_queue_.end();

    ros::Time time_begin = std::min(it_left_begin->first, it_right_begin->first);
    ros::Time time_end = std::min(prev(it_left_end, 1)->first, prev(it_right_end, 1)->first);

    it_left_end = events_left_queue_.upper_bound(time_end);
    it_right_end = events_right_queue_.upper_bound(time_end);

    //    std::cout << "left begin  " << it_left_begin->first << std::endl;
    //    std::cout << "right begin " << it_right_begin->first << std::endl;
    //    std::cout << "left end    " << it_left_begin->first << std::endl;
    //    std::cout << "right end   " << it_right_end->first << std::endl;

    ros::Time curr_time_stamp;
    curr_time_stamp.fromNSec(static_cast<uint64_t>((time_begin_.toNSec() + (time_end_ - time_begin_).toNSec() / 2.0)));

    if (std::distance(it_left_begin, it_left_end) < integration_length_ ||
        std::distance(it_right_begin, it_right_end) < integration_length_)
    {
      return;
    }

    std::cout << "event left pub :  " << std::distance(it_left_begin, it_left_end) << std::endl;
    //    std::cout << "event left left:  " << events_left_queue_.size() - std::distance(it_left_begin, it_left_end)
    //              << std::endl;
    std::cout << "event right pub : " << std::distance(it_right_begin, it_right_end) << std::endl;
    //    std::cout << "event right left: " << events_right_queue_.size() - std::distance(it_right_begin, it_right_end)
    //              << std::endl;

    if (display_method_ == RED_BLUE)
    {
      cv_image_left.encoding = "bgr8";
      cv_image_right.encoding = "bgr8";

      last_image_left_.copyTo(cv_image_left.image);
      last_image_right_.copyTo(cv_image_right.image);

      cv_image_left.header.stamp = curr_time_stamp;
      cv_image_right.header.stamp = curr_time_stamp;

      // left image
      for (std::map<ros::Time, dvs_msgs::Event>::iterator it_left = it_left_begin; it_left != it_left_end; ++it_left)
      {
        const int x = it_left->second.x;
        const int y = it_left->second.y;

        cv_image_left.image.at<cv::Vec3b>(cv::Point(x, y)) =
            (it_left->second.polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }

      // right image
      for (std::map<ros::Time, dvs_msgs::Event>::iterator it_right = it_right_begin; it_right != it_right_end;
           ++it_right)
      {
        const int x = it_right->second.x;
        const int y = it_right->second.y;

        cv_image_right.image.at<cv::Vec3b>(cv::Point(x, y)) =
            (it_right->second.polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));
      }
    }

    // grayscale
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
      for (std::map<ros::Time, dvs_msgs::Event>::iterator it_left = it_left_begin; it_left != it_left_end; ++it_left)
      {
        const int x = it_left->second.x;
        const int y = it_left->second.y;

        if (it_left->second.polarity == 1)
          //          on_events_left.at<uint8_t>(cv::Point(x, y))++;
          on_events_left.at<uint8_t>(cv::Point(x, y)) = 1;

        else
          //          off_events_left.at<uint8_t>(cv::Point(x, y))++;
          off_events_left.at<uint8_t>(cv::Point(x, y)) = 1;
      }

      // right image
      for (std::map<ros::Time, dvs_msgs::Event>::iterator it_right = it_right_begin; it_right != it_right_end;
           ++it_right)
      {
        const int x = it_right->second.x;
        const int y = it_right->second.y;

        if (it_right->second.polarity == 1)
          //          on_events_right.at<uint8_t>(cv::Point(x, y))++;
          on_events_right.at<uint8_t>(cv::Point(x, y)) = 1;
        else
          //          off_events_right.at<uint8_t>(cv::Point(x, y))++;
          off_events_right.at<uint8_t>(cv::Point(x, y)) = 1;
      }

      // scale image
      //      cv::normalize(on_events_left, on_events_left, 0, 128, cv::NORM_MINMAX, CV_8UC1);
      //      cv::normalize(off_events_left, off_events_left, 0, 127, cv::NORM_MINMAX, CV_8UC1);
      //      cv::normalize(on_events_right, on_events_right, 0, 128, cv::NORM_MINMAX, CV_8UC1);
      //      cv::normalize(off_events_right, off_events_right, 0, 127, cv::NORM_MINMAX, CV_8UC1);

      cv_image_left.image += on_events_left;
      cv_image_left.image -= off_events_left;
      cv_image_right.image += on_events_right;
      cv_image_right.image -= off_events_right;

      cv::normalize(cv_image_left.image, cv_image_left.image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(cv_image_right.image, cv_image_right.image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }

    image_left_pub_.publish(cv_image_left.toImageMsg());
    image_right_pub_.publish(cv_image_right.toImageMsg());
    events_left_queue_.erase(it_left_begin, it_left_end);
    events_right_queue_.erase(it_right_begin, it_right_end);
    events_right_.clear();

    if (got_stereo_camera_model_ &&
        ((rectified_image_left_pub_.getNumSubscribers() > 0 && rectified_image_right_pub_.getNumSubscribers() > 0) ||
         rectified_image_stereo_pub_.getNumSubscribers() > 0))
    {
      cv_bridge::CvImage cv_image2_left;
      cv_bridge::CvImage cv_image2_right;
      cv_bridge::CvImage cv_image2_stereo;

      cv_image2_left.encoding = cv_image_left.encoding;
      cv_image2_right.encoding = cv_image_right.encoding;
      cv_image2_stereo.encoding = cv_image_left.encoding;

      if (rectification_method_ == BILINEAR)
      {
        stereo_camera_model_.left().rectifyImage(cv_image_left.image, cv_image2_left.image);
        stereo_camera_model_.right().rectifyImage(cv_image_right.image, cv_image2_right.image);
      }
      else
      {
        stereo_camera_model_.left().rectifyImage(cv_image_left.image, cv_image2_left.image, cv::INTER_NEAREST);
        stereo_camera_model_.right().rectifyImage(cv_image_right.image, cv_image2_right.image, cv::INTER_NEAREST);
      }

      cv::Size size_left = cv_image2_left.image.size();
      cv::Size size_right = cv_image2_right.image.size();
      cv::Mat stereo_image(size_left.height, size_left.width + size_right.width, CV_8U);
      cv::Mat left(stereo_image, cv::Rect(0, 0, size_left.width, size_left.height));
      cv_image2_left.image.copyTo(left);
      cv::Mat right(stereo_image, cv::Rect(size_left.width, 0, size_right.width, size_right.height));
      cv_image2_right.image.copyTo(right);

      cv_image2_stereo.image = stereo_image;

      cv_image2_left.header.stamp = curr_time_stamp;
      cv_image2_right.header.stamp = curr_time_stamp;
      cv_image2_stereo.header.stamp = curr_time_stamp;

      rectified_image_left_pub_.publish(cv_image2_left.toImageMsg());
      rectified_image_right_pub_.publish(cv_image2_right.toImageMsg());
      rectified_image_stereo_pub_.publish(cv_image2_stereo.toImageMsg());
    }
  }
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

}  // namespace
