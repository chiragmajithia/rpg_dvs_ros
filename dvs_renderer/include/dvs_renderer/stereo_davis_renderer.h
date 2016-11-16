#ifndef STEREO_DAVIS_RENDERER_H
#define STEREO_DAVIS_RENDERER_H

#include <ros/ros.h>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "image_tracking.h"

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include <deque>

namespace dvs_renderer
{
class StereoDavisRenderer
{
public:
  StereoDavisRenderer(ros::NodeHandle& nh, ros::NodeHandle nh_private);
  ~StereoDavisRenderer();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void cameraInfoLeftCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void cameraInfoRightCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void eventsLeftCallback(const dvs_msgs::EventArray::ConstPtr& msg);
  void eventsRightCallback(const dvs_msgs::EventArray::ConstPtr& msg);

  void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
  void imageLeftCallback(const sensor_msgs::Image::ConstPtr& msg);
  void imageRightCallback(const sensor_msgs::Image::ConstPtr& msg);

  void publishImageAndClearEvents();
  void publishStats();

  void setStereoCameraModel();

  bool got_camera_info_;
  bool got_camera_info_left_;
  bool got_camera_info_right_;
  bool got_stereo_camera_model_;
  cv::Mat camera_matrix_, dist_coeffs_;

  //  ros::Subscriber event_sub_;
  ros::Subscriber event_left_sub_;
  ros::Subscriber event_right_sub_;

  ros::Subscriber camera_info_left_sub_;
  ros::Subscriber camera_info_right_sub_;

  image_geometry::StereoCameraModel stereo_camera_model_;

  image_transport::Publisher image_pub_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_left_sub_;
  image_transport::Subscriber image_right_sub_;

  image_transport::Publisher undistorted_image_left_pub_;
  image_transport::Publisher undistorted_image_right_pub_;
  image_transport::Publisher image_left_pub_;
  image_transport::Publisher image_right_pub_;

  cv::Mat last_image_;
  cv::Mat last_image_left_;
  cv::Mat last_image_right_;

  size_t integration_length_;
  bool use_milliseconds_;

  std::deque<dvs_msgs::Event> events_;
  std::deque<dvs_msgs::Event> events_left_;
  std::deque<dvs_msgs::Event> events_right_;

  sensor_msgs::CameraInfo camera_info_left_;
  sensor_msgs::CameraInfo camera_info_right_;

  struct EventStats
  {
    ros::Publisher events_mean_[2]; /**< event stats output */
    int events_counter_[2];         /**< event counters for on/off events */
    double events_mean_lasttime_;
    double dt;
  };
  EventStats event_stats_[2]; /**< event statistics for 1 and 5 sec */

  enum DisplayMethod
  {
    GRAYSCALE,
    RED_BLUE
  } display_method_;

  //  ImageTracking image_tracking_;
};

}  // namespace

#endif  // STEREO_DAVIS_RENDERER_H
