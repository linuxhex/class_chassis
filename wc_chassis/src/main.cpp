/* Copyright(C) Gaussian Robot. All rights reserved.
*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
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
#ifdef SETTING_PRIORITY
#include <sched.h>
#endif
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "wc_chassis_mcu.h"
#include "init.h"
#include "publish.h"
#include "action.h"
#include "parameter.h"
#include "common_function.h"

ros::Publisher ultrasonic_pub[15];
ros::Publisher yaw_pub;
ros::Publisher odom_pub;
ros::Publisher gyro_pub;
ros::Publisher remote_cmd_pub;
ros::Publisher device_pub;
ros::Publisher rotate_finished_pub;
ros::Publisher protector_pub;
ros::Publisher going_back_pub;

#define SETTING_PRIORITY

int main(int argc, char **argv) {

#ifdef SETTING_PRIORITY
    struct sched_param param;
    param.sched_priority = 99;
    if (0 != sched_setscheduler(getpid(), SCHED_RR, &param)) {
      std::cout << "set priority failed" << std::endl;
    } else {
      std::cout << "set priority succeed" << std::endl;
    }
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(0, &mask);
    if (sched_setaffinity(0, sizeof(mask), &mask) < 0) {
      std::cout << "set affinity failed" << std::endl;
    } else {
      std::cout << "set affinity succeed" << std::endl;
    }
#endif

    ROS_INFO("[wc_chassis] chassis version: 1.1.3.2");
    InitChassis(argc, argv,"wc_chassis");
    ROS_INFO("[wc_chassis] chassis init completed");

    /*********************************publish handle init ******************************/
    yaw_pub         = p_n->advertise<std_msgs::Float32>("yaw", 10);
    odom_pub        = p_n->advertise<nav_msgs::Odometry>("odom", 50);
    gyro_pub        = p_device_nh->advertise<sensor_msgs::Imu>("gyro", 50);
    remote_cmd_pub  = p_device_nh->advertise<std_msgs::UInt32>("remote_cmd", 50);
    device_pub      = p_device_nh->advertise<diagnostic_msgs::DiagnosticStatus>("device_status", 50);
    rotate_finished_pub = p_device_nh->advertise<std_msgs::Int32>("rotate_finished", 5);
    protector_pub   = p_device_nh->advertise<diagnostic_msgs::KeyValue>("protector", 50);
    going_back_pub  = p_device_nh->advertise<std_msgs::UInt32>("cmd_going_back", 50);

    for(int i=0;i<15;i++){
        if(ultrasonic->find(ultrasonic_str[i]) != std::string::npos){
            ultrasonic_pub[i] = p_n->advertise<sensor_msgs::Range>(ultrasonic_str[i].c_str(), 50);
            if(special_ultrasonic->find(ultrasonic_str[i]) != std::string::npos){
                special_ultrasonic_id[i] = i;
            }
            ultrasonic_num ++;
        }
    }

   timeval tv;
    /*********************************  主循环  ******************************/
   while (ros::ok()) {
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  1.loop start: getOdo", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    g_chassis_mcu->getOdo(g_odom_x, g_odom_y, g_odom_tha);
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  2.getCSpeed", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    g_chassis_mcu->getCSpeed(g_odom_v, g_odom_w);
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  3.getUltra", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    g_chassis_mcu->getUltra();

    gettimeofday(&tv, NULL);
    double time_now = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

    if(start_rotate_flag) {
      gettimeofday(&tv, NULL);
      ROS_INFO("[CHASSIS-MAIN] t = %.5f;  4.0.DoRotate", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
      DoRotate(rotate_finished_pub);
    } else {
      if ((time_now - last_cmd_vel_time >= max_cmd_interval) || (protector_down && (time_now - protector_start_time <= 1.0) && (m_speed_v >= 0))) {
        gettimeofday(&tv, NULL);
        ROS_INFO("[CHASSIS-MAIN] t = %.5f;  4.1.setTwoWheelSpeed(0, 0)", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
        g_chassis_mcu->setTwoWheelSpeed(0.0,0.0);
      } else {
        protector_down = 0;
        gettimeofday(&tv, NULL);
        ROS_INFO("[CHASSIS-MAIN] t = %.5f;  4.2.setTwoWheelSpeed(%lf, %lf)", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec, m_speed_v, m_speed_w);
        g_chassis_mcu->setTwoWheelSpeed(m_speed_v, m_speed_w);
      }
    }

    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  5.DoDIO", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    DoDIO(going_back_pub);
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  6.DoRemoteRet", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    DoRemoteRet();
    if (++loop_count % 2) {
      gettimeofday(&tv, NULL);
      ROS_INFO("[CHASSIS-MAIN] t = %.5f;  7.getRemoteCmd(5Hz)", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
      g_chassis_mcu->getRemoteCmd(remote_cmd_, remote_index_);
      gettimeofday(&tv, NULL);
      ROS_INFO("[CHASSIS-MAIN] t = %.5f;  8.PublishRemoteCmd(5Hz)", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
      PublishRemoteCmd(remote_cmd_pub,remote_cmd_, remote_index_);
      gettimeofday(&tv, NULL);
      ROS_INFO("[CHASSIS-MAIN] t = %.5f;  9.publishDeviceStatus(5Hz)", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
      publishDeviceStatus(device_pub);
    }
    if (loop_count % 10) {//设置遥控器id
      gettimeofday(&tv, NULL);
      ROS_INFO("[CHASSIS-MAIN] t = %.5f;  10.setRemoteID(1Hz)", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
      g_chassis_mcu->setRemoteID((unsigned char)((remote_id & 0x0f) | ((remote_speed_level_ & 0x03) << 4) | ((battery_level_ & 0x03) << 6)));
      loop_count = 0;
    }
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  11.PublishOdom", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    PublishOdom(p_odom_broadcaster,odom_pub);
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  12.PublishYaw && PublishGyro", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    PublishYaw(yaw_pub);
    PublishGyro(gyro_pub);
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  13.PublishUltrasonic", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    PublishUltrasonic(ultrasonic_pub);
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  14.publish_protector_status", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    publish_protector_status(protector_pub);
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  15.0.spin start", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    ros::spinOnce();
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  15.1.spin end; loop_sleep start", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
    p_loop_rate->sleep();
    gettimeofday(&tv, NULL);
    ROS_INFO("[CHASSIS-MAIN] t = %.5f;  16. loop end!!!", static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec);
  }

  ROS_INFO("[wc_chassis] wc_chassis has closed, now free resource!");
  freeResource();
  return 0;
}

