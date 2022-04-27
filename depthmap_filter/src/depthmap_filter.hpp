/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file depthmap_filter.hpp
 * @author W. Nicholas Greene
 * @date 2017-11-15 12:53:32 (Wed)
 */

#pragma once

#include <memory>
#include <vector>
#include <limits>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <nodelet/nodelet.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>

#ifdef FLA_HEALTH_STATUS
#include <fla_msgs/ProcessStatus.h>
#endif

namespace depthmap_filter {

/**
 * @brief Filters a depthmap.
 */
class DepthmapFilter : public nodelet::Nodelet {
 public:
  // Convenience typedefs.
  typedef message_filters::sync_policies::
  ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                  sensor_msgs::CameraInfo> RGBDPolicy;
  typedef message_filters::Synchronizer<RGBDPolicy> RGBDSynchronizer;

#ifdef FLA_HEALTH_STATUS
  enum Status {
    GOOD = 0,
    ALARM_TIMEOUT = 2,
    FAIL_TIMEOUT = 3,
  };
#endif

  /**
   * @brief Constructor.
   *
   * NOTE: Default, no-args constructor must exist.
   */
  DepthmapFilter() = default;

  virtual ~DepthmapFilter() = default;

  DepthmapFilter(const DepthmapFilter& rhs) = delete;
  DepthmapFilter& operator=(const DepthmapFilter& rhs) = delete;

  DepthmapFilter(const DepthmapFilter&& rhs) = delete;
  DepthmapFilter& operator=(const DepthmapFilter&& rhs) = delete;

  /**
   * @brief Nodelet initialization.
   *
   * Subclasses of nodelet::Nodelet need to override this virtual method.
   * It takes the place of the nodelet constructor.
   */
  virtual void onInit();

  /**
   * @brief Main image callback.
   */
  void RGBDCallback(const sensor_msgs::Image::ConstPtr& rgb_msg,
                    const sensor_msgs::Image::ConstPtr& depth_msg,
                    const sensor_msgs::CameraInfo::ConstPtr& cinfo);

#ifdef FLA_HEALTH_STATUS
  void HeartBeat(const ros::TimerEvent&);
#endif

 private:
  cv::Mat1f DownsampleImage(cv::Mat1f& original_img);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  int num_imgs_;
  int downsample_factor_;
  int pyramid_level_;

  // Gradient filter params.
  bool do_gradient_filter_;
  float max_grad_mag_;

  // Saturation filter params.
  bool do_saturation_filter_;
  int saturation_thresh_;

  // Morphological filter params.
  bool do_morph_open_;
  int morph_open_size_;
  bool do_morph_close_;
  int morph_close_size_;

  // Maximum depth.
  float max_depth_;
  float min_depth_;

  // Will use a combination of message_filters and image_transport to
  // synchronize rgb, depth, and camera info topics.
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::SubscriberFilter rgb_sub_;
  image_transport::SubscriberFilter depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  std::shared_ptr<RGBDSynchronizer> sync_;

  // Depthmap publisher.
  image_transport::CameraPublisher depth_pub_;

  // State.
  double last_update_sec_;

#ifdef FLA_HEALTH_STATUS
  int node_id_;
  double heart_beat_dt_;
  double alarm_timeout_;
  double fail_timeout_;
  ros::Timer heart_beat_;
  ros::Publisher heart_beat_pub_;
#endif
};

}  // namespace depthmap_filter

// Export as a plugin.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depthmap_filter::DepthmapFilter, nodelet::Nodelet)
