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
  check_distance_min_ = 2.0;
  single_isolated_distance_min_ = 0.05;
  double_isolated_distance_min_ = 0.07;
  triple_isolated_distance_min_ = 0.10;
  begin_skip_beams_ = 2;
  end_skip_beams_ = 2;
}

SingleLaserFilter::~SingleLaserFilter() { }

bool SingleLaserFilter::configure() {
  getParam("check_distance_min", check_distance_min_);
  getParam("single_distance_min", single_isolated_distance_min_);
  getParam("double_distance_min", double_isolated_distance_min_);
  getParam("triple_distance_min", triple_isolated_distance_min_);
  getParam("upper_bounds", upper_bounds_);
  getParam("begin_skip_beams", begin_skip_beams_);
  getParam("end_skip_beams", end_skip_beams_);
  return true;
}

bool SingleLaserFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
  scan_out = scan_in;
  for (int i = begin_skip_beams_ + 2; i < scan_out.ranges.size() - end_skip_beams_ - 3; ++i) {
//      scan_out.ranges[i] = scan_out.ranges[i - 1];
    if (scan_in.ranges[i] < check_distance_min_
        && fabs(scan_in.ranges[i] - scan_in.ranges[i - 1]) > single_isolated_distance_min_ 
        && fabs(scan_in.ranges[i] - scan_in.ranges[i + 1]) > single_isolated_distance_min_) {
      scan_out.ranges[i] = 0.0;
      continue;
    }
    if (scan_in.ranges[i] < check_distance_min_
        && fabs(scan_in.ranges[i] - scan_in.ranges[i - 1]) > double_isolated_distance_min_ 
        && fabs(scan_in.ranges[i] - scan_in.ranges[i + 2]) > double_isolated_distance_min_ 
        && fabs(scan_in.ranges[i + 1] - scan_in.ranges[i - 1]) > double_isolated_distance_min_
        && fabs(scan_in.ranges[i + 1] - scan_in.ranges[i + 2]) > double_isolated_distance_min_) {
      scan_out.ranges[i] = 0.0;
      scan_out.ranges[i + 1] = 0.0;
      i += 1;
      continue;
    }
/*    if (scan_in.ranges[i] < check_distance_min_
        && fabs(scan_in.ranges[i] - scan_in.ranges[i - 2]) > triple_isolated_distance_min_ 
        && fabs(scan_in.ranges[i] - scan_in.ranges[i + 2]) > triple_isolated_distance_min_ 
        && fabs(scan_in.ranges[i - 1] - scan_in.ranges[i - 2]) > triple_isolated_distance_min_
        && fabs(scan_in.ranges[i - 1] - scan_in.ranges[i + 2]) > triple_isolated_distance_min_ 
        && fabs(scan_in.ranges[i + 1] - scan_in.ranges[i - 2]) > triple_isolated_distance_min_
        && fabs(scan_in.ranges[i + 1] - scan_in.ranges[i + 2]) > triple_isolated_distance_min_) {
      scan_out.ranges[i] = 0.0;
      scan_out.ranges[i - 1] = 0.0;
      scan_out.ranges[i + 1] = 0.0;
      i += 2;
    }
*/
  }
  return true;
}

}  // namespace gaussian_laser_filters
