/* Copyright(C) Gaussian Robot. All rights reserved.
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <autoscrubber_services/StopScrubber.h>
#include <autoscrubber_services/StartRotate.h>
#include <autoscrubber_services/StopRotate.h>
#include <autoscrubber_services/CheckRotate.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>  // for setting priority
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "wc_chassis_mcu.h"  // NOLINT
#include "init.h"

int main(int argc, char **argv) {

  InitChassis(argc, argv);
  while (ros::ok()) {
    g_chassis_mcu->getOdo(g_odom_x, g_odom_y, g_odom_tha);
    g_chassis_mcu->getCSpeed(g_odom_v, g_odom_w);
    usleep(1000);
    timeval tv;
    gettimeofday(&tv, NULL);
    double time_now = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

    if(start_rotate_flag) {
      DoRotate();
    } else {
      if (time_now - last_cmd_vel_time >= max_cmd_interval) {
        g_chassis_mcu->setTwoWheelSpeed(0.0,0.0);
      } else {
//        pthread_mutex_lock(&speed_mutex);
//        float speed_v = g_speed_v[current_v_index];
//        float speed_w = g_speed_w[current_w_index];
//        pthread_mutex_unlock(&speed_mutex);
        g_chassis_mcu->setTwoWheelSpeed(m_speed_v, m_speed_w);
      }
    }
   // set do get di
    DoDIO();
    DoRemoteRet();
    if (++loop_count % 2) {
      g_chassis_mcu->getRemoteCmd(remote_cmd_, remote_index_);
      PublisheRemoteCmd(remote_cmd_, remote_index_);
      publish_device_status();
      loop_count = 0;
    }
    //发布里程计
    PublishOdom(odom_broadcaster);
    PublishYaw();
    PublishGyro();
    // publish ultrasonic data
    PublishUltrasonic();
    ros::spinOnce();
    loop_rate->sleep();
  }
  return 0;
}

