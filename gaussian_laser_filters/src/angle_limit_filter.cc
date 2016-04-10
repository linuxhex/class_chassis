/* Copyright(C) Gaussian Robot. All rights reserved.
*/

/**
 * @file angle_limit_filter.cc
 * @brief limit laser angle, and return range_max + 1 for beams that out range
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-07-08
 */

#include <iostream>
#include "gaussian_laser_filters/angle_limit_filter.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(gaussian_laser_filters::AngleLimitFilter, filters::FilterBase<sensor_msgs::LaserScan>)

namespace gaussian_laser_filters {

AngleLimitFilter::AngleLimitFilter() { }

AngleLimitFilter::~AngleLimitFilter() { }

bool AngleLimitFilter::configure() {
  angle_min_ = -M_PI / 2.0;
  angle_max_ = M_PI / 2.0;
  range_outside_ = 0.01;
  getParam("angle_min", angle_min_);
  getParam("angle_max", angle_max_);
  getParam("range_outside", range_outside_);
  return true;
}

bool AngleLimitFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
  if (scan_in.angle_min <= angle_min_ && scan_in.angle_max >= angle_max_) {
    scan_out = scan_in;
    double angle = scan_out.angle_min;
    for (unsigned int i = 0; i < scan_out.ranges.size(); ++i) {
      if (angle <= angle_min_ || angle >= angle_max_) {
        scan_out.ranges[i] = 0.0;
      }
      angle += scan_out.angle_increment;
    }
    // scan_out.angle_min = angle_min_;
    // scan_out.angle_max = angle_max_;
  } else {
    assert(scan_in.angle_min > angle_min_ && scan_in.angle_max < angle_max_);
    scan_out.ranges.clear();
    scan_out.intensities.clear();

    double current_angle = angle_min_;
    scan_out.header.stamp = scan_in.header.stamp;
    while (current_angle < scan_in.angle_min) {
      scan_out.ranges.push_back(range_outside_);
      if (scan_in.intensities.size() > 0)
        scan_out.intensities.push_back(scan_in.intensities.front());
      current_angle += scan_in.angle_increment;
      scan_out.header.stamp -= ros::Duration(scan_in.time_increment);
    }
    for (unsigned int i = 0; i < scan_in.ranges.size(); ++i) {
      scan_out.ranges.push_back(scan_in.ranges[i]);
      if (scan_in.intensities.size() > 0)
        scan_out.intensities.push_back(scan_in.intensities[i]);
      current_angle += scan_in.angle_increment;
    }
    while (current_angle <= angle_max_) {
      scan_out.ranges.push_back(range_outside_);
      if (scan_in.intensities.size() > 0)
        scan_out.intensities.push_back(scan_in.intensities.back());
      current_angle += scan_in.angle_increment;
    }

    // make sure to set all the needed fields on scan_out
    scan_out.header.frame_id = scan_in.header.frame_id;
    scan_out.angle_min = angle_min_;
    scan_out.angle_max = angle_max_;
    scan_out.angle_increment = scan_in.angle_increment;
    scan_out.time_increment = scan_in.time_increment;
    scan_out.scan_time = scan_in.scan_time;
    scan_out.range_min = scan_in.range_min;
    scan_out.range_max = scan_in.range_max;
    std::cout << "angle_min = " << scan_out.angle_min << "; angle_max = " << scan_out.angle_max << std::endl;
  }
  return true;
}

}  // namespace gaussian_laser_filters
