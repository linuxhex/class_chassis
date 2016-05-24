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

#define PI 3.1415926535898 

PLUGINLIB_EXPORT_CLASS(gaussian_laser_filters::SingleLaserFilter, filters::FilterBase<sensor_msgs::LaserScan>)

namespace gaussian_laser_filters {

SingleLaserFilter::SingleLaserFilter() {
  check_distance_min_ = 2.0;
  single_isolated_distance_min_ = 0.05;
  double_isolated_distance_min_ = 0.07;
  triple_isolated_distance_min_ = 0.10;
  begin_skip_beams_ = 2;
  end_skip_beams_ = 2;

  compare_frame_interval_ = 1;
  judge_dist_ = 0.02;
  find_beams_num_ = 1;
  ros::NodeHandle g_nh;
  odometry_sub_ = g_nh.subscribe("/odom", 1, &SingleLaserFilter::odometryCallback, this);
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

  getParam("compare_frame_interval", compare_frame_interval_);
  getParam("judge_dist", judge_dist_);
  getParam("find_beams_num", find_beams_num_);
  return true;
}

bool SingleLaserFilter::update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out) {
  ScanOdometryPair scan_odometry_pair;
  {
    boost::mutex::scoped_lock lock(odometry_mutex_);
    scan_odometry_pair = {scan_in, new_odometry_};
  }
  
  pair_queue_.push(scan_odometry_pair);

  scan_out = scan_in;
  if (pair_queue_.size() == compare_frame_interval_ + 1) {
    for (int i = begin_skip_beams_ + 2; i < scan_out.ranges.size() - end_skip_beams_ - 3; ++i) {
      if (scan_in.ranges[i] < scan_in.range_min || scan_in.ranges[i] > scan_in.range_max) {
        continue;
      }
      if (scan_in.ranges[i] < check_distance_min_
          && fabs(scan_in.ranges[i] - scan_in.ranges[i - 1]) > single_isolated_distance_min_ 
          && fabs(scan_in.ranges[i] - scan_in.ranges[i + 1]) > single_isolated_distance_min_) {
        scan_out.ranges[i] = 0.0;
        continue;
      }

      if (scan_in.ranges[i + 1] < scan_in.range_min || scan_in.ranges[i + 1] > scan_in.range_max) {
        i += 1;
        continue;
      }
/*
      if (scan_in.ranges[i] < check_distance_min_
          && fabs(scan_in.ranges[i] - scan_in.ranges[i - 1]) > double_isolated_distance_min_ 
          && fabs(scan_in.ranges[i] - scan_in.ranges[i + 2]) > double_isolated_distance_min_ 
          && fabs(scan_in.ranges[i + 1] - scan_in.ranges[i - 1]) > double_isolated_distance_min_
          && fabs(scan_in.ranges[i + 1] - scan_in.ranges[i + 2]) > double_isolated_distance_min_) {
        if (!judgeCurrentFrameInLastFrame(i))
          scan_out.ranges[i] = 0.0;
        if (!judgeCurrentFrameInLastFrame(i + 1))
          scan_out.ranges[i + 1] = 0.0;
        i += 1;
        continue;
      }
*/
    }
    pair_queue_.pop();
  }
  return true;
}


void SingleLaserFilter::odometryCallback(const nav_msgs::OdometryConstPtr& message) {
  boost::mutex::scoped_lock lock(odometry_mutex_);
  new_odometry_ = *message;
}

bool SingleLaserFilter::judgeCurrentFrameInLastFrame(int beam_index) {
  sensor_msgs::LaserScan current_scan = pair_queue_.back().scan;
  nav_msgs::Odometry current_odometry = pair_queue_.back().odometry;

  sensor_msgs::LaserScan last_scan = pair_queue_.front().scan;
  nav_msgs::Odometry last_odometry = pair_queue_.front().odometry;
  double x_in_current_frame = current_scan.ranges[beam_index];
  double y_in_current_frame = 0;
  double angle = current_scan.angle_min + beam_index*(current_scan.angle_increment);
  /*rotate(x_in_current_frame, y_in_current_frame,
         &x_in_current_frame, &y_in_current_frame, angle);
*/
  double x_in_last_frame = x_in_current_frame;
  double y_in_last_frame = y_in_current_frame;

  double v_x = current_odometry.pose.pose.position.x -
               last_odometry.pose.pose.position.x;
  double v_y = current_odometry.pose.pose.position.y -
               last_odometry.pose.pose.position.y;

  double current_yaw = tf::getYaw(current_odometry.pose.pose.orientation);   
  double last_yaw = tf::getYaw(last_odometry.pose.pose.orientation);  
  double v_angle = current_yaw - last_yaw;

  x_in_last_frame += v_x;
  y_in_last_frame += v_y;
  rotate(x_in_last_frame, y_in_last_frame,
         &x_in_last_frame, &y_in_last_frame, v_angle + angle);

  double angle_in_last_frame = getAngleFromVector(x_in_last_frame, y_in_last_frame);
  if (angle_in_last_frame < last_scan.angle_min ||
      angle_in_last_frame > last_scan.angle_max) {
    return false;
  } 

  int index_in_last_frame = round((angle_in_last_frame - last_scan.angle_min) / current_scan.angle_increment);
  for (int i = index_in_last_frame - find_beams_num_; i <= index_in_last_frame + find_beams_num_; ++i) {
    if (i >= 0 && i < last_scan.ranges.size()) {
      double x = last_scan.ranges[i];
      double y = 0;
      double angle = last_scan.angle_min + i*(last_scan.angle_increment);
      rotate(x, y, &x, &y, angle);
      double dist = hypot(x - x_in_last_frame, y - y_in_last_frame);
      if (dist < judge_dist_)
        return true;
    }
  }

  return false;
}


double SingleLaserFilter::getAngleFromVector(double x,double y) {
    double angle;
    if(x == 0 && y == 0)
      return 0.0;
    else if (x == 0 && y > 0)
      angle = PI / 2;
    else if(x == 0 && y < 0)
      angle = -PI / 2;
    else {
      double slope = y / (double)x;
      angle = atan(slope);
      if(x < 0) {
        if(y > 0)
          angle += PI;
        else
          angle -= PI;
      }
    }
    return angle;
  }

}  // namespace gaussian_laser_filters
