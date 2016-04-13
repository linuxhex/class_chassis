#ifndef __INIT__
#define __INIT__
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
#include <sched.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "wc_chassis_mcu.h"
 
 extern  double ACC_LIM_TH ;
 extern  double ultral_effective_range ;
 extern  double g_odom_x   ;
 extern  double g_odom_y   ;
 extern  double g_odom_tha ;
 extern  double g_odom_v ;
 extern  double g_odom_w ;

 enum Device_ID{
   Emergency_stop = 0,
   Mode,
   Battery,
   Mileage,
   Device_MAX
 };

extern unsigned int g_dio_count ;
extern unsigned int g_ret_count ;
extern unsigned int g_pc_control;
extern double last_cmd_vel_time ;
extern double max_cmd_interval ;
extern int current_v_index;
extern int current_w_index;
extern float m_speed_v;
extern float m_speed_w ;
extern float g_speed_v[3];
extern float g_speed_w[3];
extern float g_spe;
extern float g_angle;
extern unsigned int loop_count;
extern unsigned int rotate_angle;
extern bool start_rotate_flag;
extern bool stop_rotate_flag;
extern bool is_rotate_finished;
extern unsigned int g_di_data_;
extern unsigned int g_do_data_;
extern unsigned int cur_emergency_status;
extern double battery_full_level;
extern double battery_empty_level;
extern unsigned int remote_ret_;
extern unsigned int remote_ret_cnt_;
extern unsigned char remote_cmd_;
extern unsigned short remote_index_;

extern WC_chassis_mcu *g_chassis_mcu;
extern tf::TransformBroadcaster *odom_broadcaster;
extern ros::Rate *loop_rate;

/***
 *函数声明
*/
extern bool InitChassis(int argc, char **argv);
extern void DoNavigation(const geometry_msgs::Twist& Navigation_msg);
extern void RemoteRetCallback(const std_msgs::UInt32& ret);
extern void GyroUpdateCallback(const std_msgs::UInt32& state);
extern void publish_ultrasonic(ros::Publisher& publisher, const char* frame_id, int recv_int);
extern void PublishUltrasonic();
extern void PublishYaw();
extern void PublishGyro();
extern void PublishOdom(tf::TransformBroadcaster* odom_broadcaster);
extern void PublisheRemoteCmd(unsigned char cmd, unsigned short index);
extern void publish_device_status();
extern bool DoRotate();
extern void DoDIO();
extern void DoRemoteRet();
extern bool StartRotate(autoscrubber_services::StartRotate::Request& req, autoscrubber_services::StartRotate::Response& res);
extern bool StopRotate(autoscrubber_services::StopRotate::Request& req, autoscrubber_services::StopRotate::Response& res);
extern bool CheckRotate(autoscrubber_services::CheckRotate::Request& req, autoscrubber_services::CheckRotate::Response& res);

#endif



