/**
 * Copyright 2017 Massachusetts Institute of Technology
 *
 * @file depthmap_filter.cpp
 * @author W. Nicholas Greene
 * @date 2017-11-15 12:53:41 (Wed)
 */

#include "./depthmap_filter.hpp"
#include "fla_utils/param_utils.h"

#include <vector>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>

namespace depthmap_filter
{
void DepthmapFilter::onInit()
{
  // Grab a handle to the parent node.
  nh_ = getNodeHandle();
  pnh_ = getPrivateNodeHandle();

  fla_utils::SafeGetParam(pnh_, "node_id", node_id_);
  fla_utils::SafeGetParam(pnh_, "heart_beat_dt", heart_beat_dt_);
  fla_utils::SafeGetParam(pnh_, "alarm_timeout", alarm_timeout_);
  fla_utils::SafeGetParam(pnh_, "fail_timeout", fail_timeout_);
  fla_utils::SafeGetParam(pnh_, "downsample_factor", downsample_factor_);
  fla_utils::SafeGetParam(pnh_, "pyramid_level", pyramid_level_);

  // Gradient filter params.
  fla_utils::SafeGetParam(pnh_, "do_gradient_filter", do_gradient_filter_);
  fla_utils::SafeGetParam(pnh_, "max_grad_mag", max_grad_mag_);

  // Saturation filter params.
  fla_utils::SafeGetParam(pnh_, "do_saturation_filter", do_saturation_filter_);
  fla_utils::SafeGetParam(pnh_, "saturation_thresh", saturation_thresh_);

  // Morphological filter params.
  fla_utils::SafeGetParam(pnh_, "morph/do_open", do_morph_open_);
  fla_utils::SafeGetParam(pnh_, "morph/open_size", morph_open_size_);
  fla_utils::SafeGetParam(pnh_, "morph/do_close", do_morph_close_);
  fla_utils::SafeGetParam(pnh_, "morph/close_size", morph_close_size_);

  // Maximum depth threshold.
  fla_utils::SafeGetParam(pnh_, "max_depth", max_depth_);
  fla_utils::SafeGetParam(pnh_, "min_depth", min_depth_);

  // Setup subscriber.
  it_ = std::make_shared<image_transport::ImageTransport>(nh_);
  rgb_sub_.subscribe(*it_, "rgb", 1);
  depth_sub_.subscribe(*it_, "depth", 1);
  info_sub_.subscribe(nh_, "camera_info", 1);

  // Set up synchronizer.
  sync_.reset(new RGBDSynchronizer(RGBDPolicy(1), rgb_sub_, depth_sub_, info_sub_));  // RGBDPolicy(queue_size)
  sync_->registerCallback(boost::bind(&DepthmapFilter::RGBDCallback, this, _1, _2, _3));

  // Setup publisher.
  depth_pub_ = it_->advertiseCamera("depth_filtered", 10);

#ifdef FLA_HEALTH_STATUS
  heart_beat_ = nh_.createTimer(ros::Duration(heart_beat_dt_), &DepthmapFilter::HeartBeat, this);
  heart_beat_pub_ = nh_.advertise<fla_msgs::ProcessStatus>("/globalstatus", 1);
#endif

  last_update_sec_ = ros::Time::now().toSec();
  num_imgs_ = 0;

  return;
}

void DepthmapFilter::RGBDCallback(const sensor_msgs::Image::ConstPtr& rgb_msg,
                                  const sensor_msgs::Image::ConstPtr& depth_msg,
                                  const sensor_msgs::CameraInfo::ConstPtr& cinfo)
{
  num_imgs_++;
  if (num_imgs_ % downsample_factor_ != 0)
  {
    // Downsample image stream.
    return;
  }

  ros::WallTime start = ros::WallTime::now();

  NODELET_DEBUG("Received image!\n");

  // Grab gray data as OpenCV image.
  cv_bridge::CvImageConstPtr gray_ptr;
  gray_ptr = cv_bridge::toCvCopy(rgb_msg, "mono8");
  cv::Mat1b gray(gray_ptr->image);

  // Grab depth data as OpenCV image.
  cv_bridge::CvImageConstPtr depth_ptr;
  depth_ptr = cv_bridge::toCvCopy(depth_msg, "32FC1");
  cv::Mat1f depthmap(depth_ptr->image);

  if (depth_msg->encoding == "16UC1")
  {
    depthmap /= 1000;  // Convert to meters.
  }

  int height = depthmap.rows;
  int width = depthmap.cols;
  cv::Mat1f depthmap_filt(height, width, 0.0f);

  if (do_gradient_filter_)
  {
    // Apply simple derivative filter to remove smearing across depth
    // discontinuities.
    float grad_thresh2 = max_grad_mag_ * max_grad_mag_;
    for (int ii = 1; ii < height - 1; ++ii)
    {
      for (int jj = 1; jj < width - 1; ++jj)
      {
        if ((depthmap(ii, jj + 1) <= 0) || (depthmap(ii, jj - 1) <= 0) || (depthmap(ii + 1, jj) <= 0) ||
            (depthmap(ii - 1, jj) <= 0))
        {
          continue;
        }

        // Compute central difference.
        float gx = 0.5f * (depthmap(ii, jj + 1) - depthmap(ii, jj - 1));
        float gy = 0.5f * (depthmap(ii + 1, jj) - depthmap(ii - 1, jj));

        float grad_mag2 = gx * gx + gy * gy;
        if (grad_mag2 < grad_thresh2)
        {
          depthmap_filt(ii, jj) = depthmap(ii, jj);
        }
      }
    }
  }
  else
  {
    depthmap_filt = depthmap;
  }

  if (do_saturation_filter_)
  {
    // Remove oversaturated regions.
    cv::Mat1b sat_mask(height, width, static_cast<uint8_t>(0));
    for (int ii = 0; ii < height; ++ii)
    {
      for (int jj = 0; jj < width; ++jj)
      {
        if (gray(ii, jj) >= saturation_thresh_)
        {
          sat_mask(ii, jj) = 255;
        }
      }
    }

    // cv::imshow("gray", gray);
    // cv::imshow("sat_mask", sat_mask);
    // cv::waitKey(1);

    for (int ii = 0; ii < height; ++ii)
    {
      for (int jj = 0; jj < width; ++jj)
      {
        if (sat_mask(ii, jj) > 0)
        {
          depthmap_filt(ii, jj) = 0.0f;
        }
      }
    }
  }

  if (do_morph_open_)
  {
    // Apply opening operator to remove speckle noise.
    cv::Mat struct_el(morph_open_size_, morph_open_size_, cv::DataType<uint8_t>::type, cv::Scalar(1));
    cv::morphologyEx(depthmap_filt, depthmap_filt, cv::MORPH_OPEN, struct_el);
  }

  if (do_morph_close_)
  {
    // Apply closing operator to connect fragmented components.
    cv::Mat struct_el(morph_close_size_, morph_close_size_, cv::DataType<uint8_t>::type, cv::Scalar(1));
    cv::morphologyEx(depthmap_filt, depthmap_filt, cv::MORPH_CLOSE, struct_el);
  }

  if (max_depth_ > 0.0f)
  {
    // Clip depths beyond max depth.
    for (int ii = 0; ii < height; ++ii)
    {
      for (int jj = 0; jj < width; ++jj)
      {
        if ((depthmap_filt(ii, jj) >= max_depth_))
        {
          depthmap_filt(ii, jj) = std::numeric_limits<float>::infinity();
        }

        if (std::fabs(depthmap_filt(ii, jj)) < 1e-6)
        {
          depthmap_filt(ii, jj) = std::numeric_limits<float>::quiet_NaN();
        }

        if ((depthmap_filt(ii, jj) <= min_depth_))
        {
          depthmap_filt(ii, jj) = -std::numeric_limits<float>::infinity();
        }
      }
    }
  }

  int binning = 1;
  for (int i = 0; i < pyramid_level_; i++)
  {
    depthmap_filt = DownsampleImage(depthmap_filt);
    binning *= 2;
  }

  // Publish filtered depthmap.
  sensor_msgs::CameraInfo downsampled_cinfo = *cinfo;
  downsampled_cinfo.binning_x = binning;
  downsampled_cinfo.binning_y = binning;
  cv_bridge::CvImage depth_cvb(depth_msg->header, "32FC1", depthmap_filt);
  depth_pub_.publish(*depth_cvb.toImageMsg(), downsampled_cinfo);

  ros::WallDuration runtime = ros::WallTime::now() - start;
  NODELET_DEBUG("DepthmapFilter/ImageCallback = %f ms\n", runtime.toSec() * 1000);
  printf("DepthmapFilter/ImageCallback = %f ms\n", runtime.toSec() * 1000);

  last_update_sec_ = ros::Time::now().toSec();

  return;
}

cv::Mat1f DepthmapFilter::DownsampleImage(cv::Mat1f& original_img)
{
  int downsampled_height = original_img.rows >> 1;
  int downsampled_width = original_img.cols >> 1;
  cv::Mat1f downsampled_img(downsampled_height, downsampled_width, 0.0f);
  // //downsample
  for (int ii = 0; ii < downsampled_height; ++ii)
  {
    for (int jj = 0; jj < downsampled_width; ++jj)
    {
      int parent_ii = ii << 1;
      int parent_jj = jj << 1;
      int nan_count = 0;
      int inf_count = 0;
      int neg_inf_count = 0;
      int valid_depth_count = 0;
      float depths[4] = { original_img(parent_ii, parent_jj), original_img(parent_ii + 1, parent_jj),
                          original_img(parent_ii, parent_jj + 1), original_img(parent_ii + 1, parent_jj + 1) };
      float total_depth = 0.0f;
      float min_valid_depth = std::numeric_limits<float>::infinity();

      for (int i = 0; i < 4; i++)
      {
        if (depths[i] != depths[i])
        {
          nan_count++;
        }
        else if (!std::isfinite(depths[i]))
        {
          if (depths[i] < 0)
          {
            neg_inf_count++;
          }
          else
          {
            inf_count++;
          }
        }
        else
        {
          total_depth += depths[i];
          valid_depth_count++;
          if (depths[i] < min_valid_depth)
          {
            min_valid_depth = depths[i];
          }
        }
      }
      if (valid_depth_count > 0)
      {
        downsampled_img(ii, jj) = total_depth / valid_depth_count;
      }
      else if (neg_inf_count > 0)
      {
        downsampled_img(ii, jj) = -std::numeric_limits<float>::infinity();
      }
      else if (inf_count > 0)
      {
        downsampled_img(ii, jj) = std::numeric_limits<float>::infinity();
      }
      else
      {
        downsampled_img(ii, jj) = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  return downsampled_img;
}

#ifdef FLA_HEALTH_STATUS
void DepthmapFilter::HeartBeat(const ros::TimerEvent&)
{
  double now = ros::Time::now().toSec();

  fla_msgs::ProcessStatus::Ptr ps(new fla_msgs::ProcessStatus);

  ps->id = static_cast<uint8_t>(node_id_);
  ps->pid = getpid();

  if (now - last_update_sec_ > alarm_timeout_)
  {
    ps->status = fla_msgs::ProcessStatus::ALARM;
    ps->arg = Status::ALARM_TIMEOUT;  // Time since last update longer than expected.
  }
  else if (now - last_update_sec_ > fail_timeout_)
  {
    ps->status = fla_msgs::ProcessStatus::FAIL;
    ps->arg = Status::FAIL_TIMEOUT;  // Time since last update probably error.
  }
  else
  {
    ps->status = fla_msgs::ProcessStatus::READY;
    ps->arg = Status::GOOD;  // All good.
  }

  heart_beat_pub_.publish(ps);

  return;
}
#endif

}  // namespace depthmap_filter
