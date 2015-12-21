/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file front_safe_filter.h
 * @brief publish front safe topic
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-07-08
 */

#ifndef GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_FRONT_SAFE_FILTER_H_
#define GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_FRONT_SAFE_FILTER_H_

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "filters/filter_base.h"

namespace gaussian_laser_filters {

class FrontSafeFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
 public:
  FrontSafeFilter();
  ~FrontSafeFilter();

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
  /**
   * @brief cmd_vel topic callback
   *
   * @param Navigation_msg
   */
  void CmdVelCallback(const geometry_msgs::Twist& msg);
  /**
   * @brief publish if front safe
   *
   * @param front_safe: true -> safe, false -> not safe
   */
  void PublishFrontSafe(bool front_safe);


 private:
  double vtheta_;        /* direction of wheel */
  double vx_;            /* speed of wheel */
  double safe_sector_;   /* sector that need to check if safe */
  double safe_speed_;    /* min safe speed */
  double safe_dis_;      /* safe distance for speed less than safe_speed */
  double safe_speed_k_;  /* k of safe_speed */

  ros::Publisher front_safe_pub_;  /* publish if front is safe */
};

};  // namespace gaussian_laser_filters

#endif  // GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_FRONT_SAFE_FILTER_H_
