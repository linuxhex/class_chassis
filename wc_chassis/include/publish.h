#ifndef __PUBLISH__
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
#define __PUBLISH__

extern void PublishYaw(ros::Publisher &yaw_pub);
extern void PublishOdom(tf::TransformBroadcaster* odom_broadcaster,ros::Publisher &odom_pub );
extern void publishDeviceStatus(ros::Publisher &device_pub);
extern void PublishRemoteCmd(ros::Publisher &remote_cmd_pub,unsigned char cmd, unsigned short index);
extern void publish_ultrasonic(ros::Publisher& publisher, const char* frame_id, int recv_int);
extern void PublishUltrasonic(ros::Publisher *ultrasonic_pubs);
extern void PublishGyro(ros::Publisher &gyro_pub);

#endif
