/* Copyright(C) Gaussian Robot. All rights reserved.
*/

/**
 * @file lms1xx_filter.cc
 * @brief filter for lms1xx laser specific use
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-07-08
 */

#include "gaussian_laser_filters/lms1xx_filter.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gaussian_laser_filters::LMS1xxFilter, filters::FilterBase<sensor_msgs::LaserScan>)

namespace gaussian_laser_filters {

LMS1xxFilter::LMS1xxFilter() { }

LMS1xxFilter::~LMS1xxFilter() { }

bool LMS1xxFilter::configure() {
  zero_range_ = 5;
  lower_bounds_ = -1.0;
  upper_bounds_ = -1.0;
  getParam("zero_range", zero_range_);
  getParam("too_short_range", too_short_range_);
  getParam("lower_bounds", lower_bounds_);
  getParam("upper_bounds", upper_bounds_);
  return true;
}

bool LMS1xxFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
  scan_out = scan_in;
  // if (lower_bounds_ != -1.0) scan_out.range_min = lower_bounds_;
  // if (upper_bounds_ != -1.0) scan_out.range_max = upper_bounds_;

  for (unsigned int i = 0; i < scan_out.ranges.size(); ++i) {
    if (scan_out.ranges[i] == 0.0) {
      scan_out.ranges[i] = zero_range_;
    }
    if (scan_out.ranges[i] < scan_out.range_min) {
       scan_out.ranges[i] = too_short_range_;
    }
    if (lower_bounds_ != -1.0 && scan_out.ranges[i] < lower_bounds_) {
      scan_out.ranges[i] = lower_bounds_;
    }
    if (upper_bounds_ != -1.0 && scan_out.ranges[i] > upper_bounds_) {
       scan_out.ranges[i] = upper_bounds_;
    }
  }
  return true;
}

}  // namespace gaussian_laser_filters
