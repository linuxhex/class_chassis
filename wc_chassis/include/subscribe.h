#ifndef __SUBSCRIBE__
#define __SUBSCRIBE__

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

extern void DoNavigationCallback(const geometry_msgs::Twist& Navigation_msg);
extern void RemoteRetCallback(const std_msgs::UInt32& ret);
extern void GyroUpdateCallback(const std_msgs::UInt32& state);
extern void ShutdownCallback(const std_msgs::UInt32& cmd);

#endif
