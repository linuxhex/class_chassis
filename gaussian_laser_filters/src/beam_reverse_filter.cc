/* Copyright(C) Gaussian Robot. All rights reserved.
 */

/**
 * @file beam_reverse_filter.cc
 * @brief laser filter that reverse laser beams
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-07-08
 */

#include "gaussian_laser_filters/beam_reverse_filter.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gaussian_laser_filters::BeamReverseFilter, filters::FilterBase<sensor_msgs::LaserScan>)

namespace gaussian_laser_filters {

BeamReverseFilter::BeamReverseFilter() { }

BeamReverseFilter::~BeamReverseFilter() { }

bool BeamReverseFilter::configure() {
  return true;
}

bool BeamReverseFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
  scan_out = scan_in;
  unsigned int beam_count = scan_in.ranges.size();
  for (unsigned int i = 0; i < beam_count; ++i) {
    scan_out.ranges[i] = scan_in.ranges[beam_count - 1 - i];
  }
  return true;
}

}  // namespace gaussian_laser_filters
