/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file angle_limit_filter.h
 * @brief limit laser angle, and return range_max + 1 for beams that out range
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-07-08
 */

#ifndef GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_ANGLE_LIMIT_FILTER_H_
#define GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_ANGLE_LIMIT_FILTER_H_

#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

namespace gaussian_laser_filters {

class AngleLimitFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
 public:
  AngleLimitFilter();
  ~AngleLimitFilter();

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
  double angle_min_;  /* min range of angle */
  double angle_max_;  /* max range of angle */
  double range_outside_;  /* ranges of beams outside angle range */
};

};  // namespace gaussian_laser_filters

#endif  // GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_ANGLE_LIMIT_FILTER_H_
