/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file lms1xx_filter.h
 * @brief filter for lms1xx laser specific use
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-07-08
 */

#ifndef GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_2_LMS1XX_FILTER_H_
#define GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_2_LMS1XX_FILTER_H_

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "filters/filter_base.h"
#include <tf/tf.h>
#include <boost/thread/thread.hpp>
#include <queue> 

namespace gaussian_laser_filters {

struct ScanOdometryPair {
  sensor_msgs::LaserScan scan;
  nav_msgs::Odometry odometry;
};
class SingleLaserFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
 public:
  SingleLaserFilter();
  ~SingleLaserFilter();

  /**
   * @brief configure function
   *
   * @return success or not
   */
  bool configure();
  /**
   * @brief Update the filter and get the response
   *
   * @param scan_in The new scan to filter
   * @param scan_out The filtered scan
   *
   * @return success or not
   */
  bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);  // NOLINT

 private:

  void odometryCallback(const nav_msgs::OdometryConstPtr& message);
  bool judgeCurrentFrameInLastFrame(int beam_index);
  inline void rotate(double r_x, double r_y, double *x, double *y, double yaw) {
    *x = -r_y * sin(yaw) + r_x * cos(yaw);
    *y = r_x * sin(yaw) + r_y * cos(yaw);
  }

  double getAngleFromVector(double x,double y);

  double check_distance_min_;     /* min distance to filter */
  double single_isolated_distance_min_;   /* min distance to determain if it is isolated */
  double double_isolated_distance_min_;
  double triple_isolated_distance_min_;
  double upper_bounds_;           /* upper bounds of ranges, will replace range if longer than this */
  double begin_skip_beams_;
  double end_skip_beams_;

  int compare_frame_interval_;
  double judge_dist_;
  int find_beams_num_;

  ros::Subscriber odometry_sub_;
  std::queue<ScanOdometryPair> pair_queue_;
  nav_msgs::Odometry new_odometry_;
  boost::mutex odometry_mutex_;
};

};  // namespace gaussian_laser_filters

#endif  // GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_2_LMS1XX_FILTER_H_
