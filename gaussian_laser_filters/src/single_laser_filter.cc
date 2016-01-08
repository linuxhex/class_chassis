/* Copyright(C) Gaussian Robot. All rights reserved.
*/

/**
 * @file single_laser_filter.cc
 * @brief filter for single scan specific use
 * @author lz<lz@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-12-21
 */

#include "gaussian_laser_filters/single_laser_filter.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gaussian_laser_filters::SingleLaserFilter, filters::FilterBase<sensor_msgs::LaserScan>)

namespace gaussian_laser_filters {

SingleLaserFilter::SingleLaserFilter() {
  distance_min_ = 0.01;
}

SingleLaserFilter::~SingleLaserFilter() { }

bool SingleLaserFilter::configure() {
  getParam("distance_min", distance_min_);
  return true;
}

bool SingleLaserFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
  scan_out = scan_in;
  for (unsigned int i = 1; i < scan_out.ranges.size() - 1; ++i) {
//      scan_out.ranges[i] = scan_out.ranges[i - 1];
    if (fabs(scan_out.ranges[i] - scan_out.ranges[i - 1]) > distance_min_ &&
          fabs(scan_out.ranges[i] - scan_out.ranges[i + 1]) > distance_min_) {
      scan_out.ranges[i] = scan_out.ranges[i - 1];
    }
  }
  return true;
}

}  // namespace gaussian_laser_filters
