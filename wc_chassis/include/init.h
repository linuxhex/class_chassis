#ifndef __INIT__
#define __INIT__
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "gs/file_util.h"
#include <vector>


extern tf::TransformBroadcaster *p_odom_broadcaster;
extern ros::Rate *p_loop_rate;
extern ros::NodeHandle *p_n;
extern ros::NodeHandle *p_nh;
extern ros::NodeHandle *p_device_nh;

bool InitChassis(int argc, char **argv,const char *node_name);
void InitDevice(void);
void InitParameter(void);
void InitService(void);
void InitSchedule(void);
#endif



