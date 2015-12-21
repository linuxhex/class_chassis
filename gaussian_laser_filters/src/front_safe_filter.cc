/* Copyright(C) Gaussian Robot. All rights reserved.
*/

/**
 * @file front_safe_filter.cc
 * @brief publish front safe topic
 * @author cameron<chenkan@gs-robot.com>
 * @version 1.0.0.0
 * @date 2015-07-08
 */

#include "gaussian_laser_filters/front_safe_filter.h"

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/UInt32.h>
#include <gslib/gslib.h>
#include <math.h>

PLUGINLIB_EXPORT_CLASS(gaussian_laser_filters::FrontSafeFilter, filters::FilterBase<sensor_msgs::LaserScan>)

namespace gaussian_laser_filters {

FrontSafeFilter::FrontSafeFilter() : vx_(0.0), vtheta_(0.0) {
  ros::NodeHandle nh;
  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 100, &FrontSafeFilter::CmdVelCallback, this);
  front_safe_pub_ = nh.advertise<std_msgs::UInt32>("front_safe", 40);
}

FrontSafeFilter::~FrontSafeFilter() { }

bool FrontSafeFilter::configure() {
  safe_sector_ = M_PI / 3.0;
  safe_speed_ = 0.1;
  safe_dis_ = 1.0;
  safe_speed_k_ = 3.0;
  getParam("safe_sector", safe_sector_);
  getParam("safe_speed", safe_speed_);
  getParam("safe_dis", safe_dis_);
  getParam("safe_speed_k", safe_speed_k_);
  return true;
}

bool FrontSafeFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
  scan_out = scan_in;
  if (vx_ < 0) return true;
  double angle = scan_out.angle_min - scan_out.angle_increment;
  for (unsigned int i = 0; i < scan_out.ranges.size(); ++i) {
    angle += scan_out.angle_increment;
    // ignore distance too short
    if (fabs(scan_out.ranges[i]) < GS_DOUBLE_PRECISION) {
      continue;
    }
    // ignore angle that not in safe_sector
    if ((angle < (vtheta_ - safe_sector_ / 2.0)) || (angle > vtheta_ + safe_sector_ / 2.0)) {
      continue;
    }
    double safe_dis = 0.0;
    // compute distance to check safe, safe_dis is longer if velocity is higher
    if (vx_ < safe_speed_) {
      safe_dis = safe_dis_;
    } else {
      safe_dis = (vx_ - safe_speed_) * safe_speed_k_ + safe_dis_;
    }

    // check safe distance
    if (scan_out.ranges[i] < safe_dis) {
      PublishFrontSafe(false);
      return true;
    }
  }
  PublishFrontSafe(true);
  return true;
}

void FrontSafeFilter::PublishFrontSafe(bool front_safe) {
  std_msgs::UInt32 msg;
  msg.data = front_safe ? 0 : 1;
  front_safe_pub_.publish(msg);
}

void FrontSafeFilter::CmdVelCallback(const geometry_msgs::Twist& msg) {
  vx_     = msg.linear.x;
  vtheta_ = msg.angular.z;
}

}  // namespace gaussian_laser_filters
