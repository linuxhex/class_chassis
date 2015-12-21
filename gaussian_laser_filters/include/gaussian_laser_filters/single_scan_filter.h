/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file lms1xx_filter.h
 * @brief filter for lms1xx laser specific use
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-07-08
 */

#ifndef GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_LMS1XX_FILTER_H_
#define GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_LMS1XX_FILTER_H_

#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"

namespace gaussian_laser_filters {

class SingleScanFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
 public:
  SingleScanFilter();
  ~SingleScanFilter();

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
  double distance_min_;       /* range to replace 0.0 range */
};

};  // namespace gaussian_laser_filters

#endif  // GAUSSIAN_LASER_FILTERS_INCLUDE_GAUSSIAN_LASER_FILTERS_LMS1XX_FILTER_H_
