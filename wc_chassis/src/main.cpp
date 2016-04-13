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
#include "tools.h"

WC_chassis_mcu g_chassis_mcu;

double ACC_LIM_TH = 3.0 / 2.0 * M_PI;
double ultral_effective_range = 0.4;
double g_odom_x   = 0;
double g_odom_y   = 0;
double g_odom_tha = 0;

double g_odom_v = 0;
double g_odom_w = 0;
std::vector<int> g_ultrasonic;
/********************
  byte     0        1-13          14-17            18               19           20           21          22        23      24
       protector ultrasonic   reserve_ultra  hardware_status   device_status  battery_H    battery_L   charge_H   charge_L
 *******************/

unsigned int g_dio_count  = 0;
unsigned int g_ret_count  = 0;
unsigned int g_pc_control = 0;

double last_cmd_vel_time = 0.0;
double max_cmd_interval  = 1.0;

int current_v_index = 0;
int current_w_index = 0;
float m_speed_v = 0.0;
float m_speed_w = 0.0;
float g_speed_v[3] = {0.0, 0.0, 0.0};
float g_speed_w[3] = {0.0, 0.0, 0.0};
float g_spe = 0.0;
float g_angle = 0.0;
pthread_mutex_t speed_mutex;

unsigned int loop_count = 0;
unsigned int rotate_angle = 0;

bool start_rotate_flag = false;
bool stop_rotate_flag = true;
bool is_rotate_finished = false;

unsigned int g_di_data_ = 0;
unsigned int g_do_data_ = 0;
unsigned int cur_emergency_status = 1;
double battery_full_level;
double battery_empty_level;

unsigned char remote_cmd_;
unsigned short remote_index_;

enum Device_ID{
  Emergency_stop = 0,
  Mode,
  Battery,
  Mileage,
  Device_MAX
};

ros::Publisher odom_pub;
ros::Publisher gyro_pub;
ros::Publisher remote_cmd_pub;
ros::Publisher device_pub;
ros::Publisher yaw_pub;
ros::Publisher ultrasonic0_pub;
ros::Publisher ultrasonic1_pub;
ros::Publisher ultrasonic2_pub;
ros::Publisher ultrasonic3_pub;
ros::Publisher ultrasonic4_pub;
ros::Publisher ultrasonic5_pub;

ros::ServiceServer start_rotate_srv;
ros::ServiceServer stop_rotate_srv;
ros::ServiceServer check_rotate_srv;
//#define SETTING_PRIORITY

void DoNavigation(const geometry_msgs::Twist& Navigation_msg) {
  timeval tv;
  gettimeofday(&tv, NULL);
  last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

  m_speed_v = Navigation_msg.linear.x;
  m_speed_w = Navigation_msg.angular.z;

//  ROS_INFO("Navigation.linear.x = %f, angular.z = %f", m_speed_v, m_speed_w);

  pthread_mutex_lock(&speed_mutex);
  double temp_v = Navigation_msg.linear.x;
  double temp_w = -1 * Navigation_msg.angular.z;
//  ROS_INFO("Navigation.linear.x = %f, angular.z = %f", temp_v, temp_w);
  int temp_v_index = (current_v_index + 1) % 3;
  int temp_w_index = (current_w_index + 1) % 3;
  g_speed_v[temp_v_index] = static_cast<float>(temp_v);
  g_speed_w[temp_w_index] = static_cast<float>(temp_w);
  current_v_index = temp_v_index;
  current_w_index = temp_w_index;
  pthread_mutex_unlock(&speed_mutex);
}


void GyroUpdateCallback(const std_msgs::UInt32& state) {
  if (state.data == 0) {
    g_chassis_mcu.gyro_state_ = 0;
  } else {
    g_chassis_mcu.gyro_state_ = 1;
  }
  ROS_INFO("[wc_chassis] set gyro state = %d", g_chassis_mcu.gyro_state_);
}

void publish_ultrasonic(ros::Publisher& publisher, const char* frame_id, int recv_int) {  // NOLINT
  sensor_msgs::Range range;
  range.header.seq = 0;
  range.header.stamp = ros::Time::now();
  range.header.frame_id = frame_id;

  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.field_of_view = M_PI / 90.0;
  range.min_range = 0.04;
  range.max_range = 1.0;

  float dis_meter = recv_int * 5.44 / 1000.0;
  if (dis_meter < range.min_range) {
    range.range = range.min_range;
  } else if (dis_meter > ultral_effective_range) {  // effective range
    range.range = range.max_range;
  } else {
    range.range = dis_meter;
  }
  publisher.publish(range);
}

void PublishUltrasonic() {
  publish_ultrasonic(ultrasonic0_pub, "ultrasonic0", g_ultrasonic[1]);
  publish_ultrasonic(ultrasonic1_pub, "ultrasonic1", g_ultrasonic[2]);
  publish_ultrasonic(ultrasonic2_pub, "ultrasonic2", g_ultrasonic[3]);
  publish_ultrasonic(ultrasonic3_pub, "ultrasonic3", g_ultrasonic[4]);
  publish_ultrasonic(ultrasonic4_pub, "ultrasonic4", g_ultrasonic[5]);
  publish_ultrasonic(ultrasonic5_pub, "ultrasonic5", g_ultrasonic[6]);
}

void PublishYaw(){
  std_msgs::Float32 msg;
  msg.data = g_odom_tha * 180.0 / M_PI;
  yaw_pub.publish(msg);
}

void PublishGyro() {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "world";

  tf::Quaternion temp;
  double roll, pitch, yaw;
  roll = g_chassis_mcu.roll_angle_ / 10.0 / 180.0 * M_PI;
  pitch = g_chassis_mcu.pitch_angle_ / 10.0 / 180.0 * M_PI;
  yaw = g_chassis_mcu.yaw_angle_ / 10.0 / 180.0 * M_PI;
  temp.setRPY(roll, pitch, yaw);
//#ifdef DEBUG_PRINT
//  std::cout << "roll = " << roll * 57.6 << "; pitch = " << pitch << ";yaw = " << std::endl;
//#endif
  imu_msg.orientation.x = temp.getX();
  imu_msg.orientation.y = temp.getY();
  imu_msg.orientation.z = temp.getZ();
  imu_msg.orientation.w = temp.getW();
  gyro_pub.publish(imu_msg);
}

void PublishOdom(tf::TransformBroadcaster* odom_broadcaster) {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();;

  odom.header.frame_id = "base_odom";
  // set the position
  odom.pose.pose.position.x = g_odom_x;
  odom.pose.pose.position.y = g_odom_y;
  odom.pose.pose.position.z = 0.0;
  int i;
  for (i = 0; i < 36; i++) odom.pose.covariance.elems[i] = 0.0;
  odom.pose.covariance.elems[0]  = 1.0;
  odom.pose.covariance.elems[7]  = 1.0;
  odom.pose.covariance.elems[14] = 1.0;
  odom.pose.covariance.elems[21] = 1.0;
  odom.pose.covariance.elems[28] = 1.0;
  odom.pose.covariance.elems[35] = 1.0;

  for (i = 0; i < 36; i++) odom.twist.covariance.elems[i] = 0.0;
  odom.twist.covariance.elems[0]  = 1.0;
  odom.twist.covariance.elems[7]  = 1.0;
  odom.twist.covariance.elems[14] = 1.0;
  odom.twist.covariance.elems[21] = 1.0;
  odom.twist.covariance.elems[28] = 1.0;
  odom.twist.covariance.elems[35] = 1.0;

  geometry_msgs::Quaternion odom_quat;
//  ROS_INFO("Odo Yaw = %lf", g_odom_tha * 57.3);
  odom_quat = tf::createQuaternionMsgFromYaw(g_odom_tha);
#ifdef DEBUG_PRINT
  std::cout << "odo based" << std::endl;
  std::cout << "x : " << Odomsg.x \
      << " y : " << Odomsg.y \
      << " angle : " << Odomsg.theta*180.0/3.1415926\
      << "Current Time-> " << now_s->tm_hour << ":" << now_s->tm_min << ":" << now_s->tm_sec << std::endl;
#endif

  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = g_odom_v;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = g_odom_w;
  // publish the message
  odom_pub.publish(odom);

  tf::Quaternion q;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
  tf::Transform odom_meas(q, tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0));
  tf::StampedTransform odom_transform(odom_meas, ros::Time::now(), "base_odom", "base_link");
  odom_broadcaster->sendTransform(odom_transform);
}

void PublisheRemoteCmd(unsigned char cmd, unsigned short index) {
  std_msgs::UInt32 remote_cmd;
  if (cmd > 0 && cmd < 12) {
    ROS_INFO("[wc_chassis] get remote cmd = %d, index = %d", cmd, index);
  }
  remote_cmd.data = (index << 8) | cmd;
  remote_cmd_pub.publish(remote_cmd);
}

void publish_device_status() {
  diagnostic_msgs::DiagnosticStatus device_status;
  diagnostic_msgs::KeyValue device_value;
  device_status.level = diagnostic_msgs::DiagnosticStatus::OK;
  device_status.name = std::string("device_status");
  device_status.message = std::string("status_msgs");
  device_status.hardware_id = std::string("status_msgs");

  device_value.key = std::string("emergency_stop");
  device_value.value = cur_emergency_status == 0 ? std::string("true") : std::string("false");
  device_status.values.push_back(device_value);

  unsigned int battery_ADC = (g_ultrasonic[20] << 2) | (g_ultrasonic[21] & 0x03);
  double battery_value = (0.2393 * battery_ADC - 125.04) * 100;
  int battery_capacity;
  if(battery_value <= battery_empty_level) {
    battery_capacity = 0;
  } else if(battery_value >= battery_full_level) {
    battery_capacity = 100;
  } else {
    battery_capacity = (battery_value - battery_empty_level) / (battery_full_level - battery_empty_level) * 100;
  }
/*
  } else if(battery_value >= 2350) {
    battery_capacity = (battery_value - 2350) * 40 / (2750 - 2350) + 60;
  } else {
    battery_capacity = (battery_value - 2000) * 60 / (3500 - 3100);
  }
  battery_capacity = battery_capacity > 100 ? 100 : battery_capacity;
  battery_capacity = battery_capacity < 0 ? 0 : battery_capacity;
*/
  std::cout << "battery_ADC " << battery_ADC << "; battery_balue " << battery_value << "; battery_capacity " << battery_capacity << std::endl;
  device_value.key = std::string("battery");
  device_value.value = std::to_string(battery_capacity);
  device_status.values.push_back(device_value);

  device_value.key = std::string("mileage");
  unsigned int mileage = (unsigned int)((g_chassis_mcu.mileage_left_ + g_chassis_mcu.mileage_left_) / 2);
  device_value.value = std::to_string(mileage);
  device_status.values.push_back(device_value);

  device_pub.publish(device_status);
}

bool DoRotate() {
  if (fabs(g_chassis_mcu.acc_odom_theta_) >= fabs(rotate_angle / 180.0 * M_PI * 0.98) ) {
    start_rotate_flag = false;
    is_rotate_finished = true;
    g_chassis_mcu.setTwoWheelSpeed(0.0, 0.0);
//    ROS_INFO("[wc_chassis] rotate finished!");
    timeval tv;
    gettimeofday(&tv, NULL);
    last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
  } else {
    is_rotate_finished = false;
//    ROS_INFO("[wc_chassis] inplace rotation: cur_yaw = %lf, targer_yaw = %d ", g_chassis_mcu.acc_odom_theta_ * 57.3, rotate_angle);
    if (rotate_angle > 0) {
      g_chassis_mcu.setTwoWheelSpeed(0.0, 0.2);
//      ROS_INFO("[wc_chassis] rotate to left");
    } else if (rotate_angle < 0) {
      g_chassis_mcu.setTwoWheelSpeed(0.0, -0.2);
//      ROS_INFO("[wc_chassis] rotate to right");
    } else {
      is_rotate_finished = true;
      start_rotate_flag = false;
      g_chassis_mcu.setTwoWheelSpeed(0.0, 0.0);
    }
  }
  return true;
}

void DoDIO() {
  if ((g_dio_count++ % 2) == 0) {
    g_di_data_ = g_chassis_mcu.doDIO(g_do_data_);
    cur_emergency_status = (g_di_data_ >> (8 + Emergency_stop)) & 0x01;
  }
}


bool StartRotate(autoscrubber_services::StartRotate::Request& req, autoscrubber_services::StartRotate::Response& res) {
  rotate_angle = req.rotateAngle.data;
  start_rotate_flag = true;
  is_rotate_finished = false;
  g_chassis_mcu.acc_odom_theta_ = 0.0;
  return true;
}

bool StopRotate(autoscrubber_services::StopRotate::Request& req, autoscrubber_services::StopRotate::Response& res) {
  start_rotate_flag = false;
  return true;
}

bool CheckRotate(autoscrubber_services::CheckRotate::Request& req, autoscrubber_services::CheckRotate::Response& res) {
  res.isFinished.data = is_rotate_finished;
  return true;
}



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
<<<<<<< HEAD
        g_chassis_mcu.setTwoWheelSpeed(m_speed_v, m_speed_w);
=======
        g_chassis_mcu->setTwoWheelSpeed(m_speed_v, m_speed_w);
>>>>>>> 8e0cd9742c3d0b6b9dfba2ddf925a0b7b33bc109
      }
    }
    DoDIO();
    DoRemoteRet();
    if (++loop_count % 2) {
      g_chassis_mcu->getRemoteCmd(remote_cmd_, remote_index_);
      PublisheRemoteCmd(remote_cmd_, remote_index_);
      publish_device_status();
      loop_count = 0;
    }
<<<<<<< HEAD
    //发布里程计
    PublishOdom(&odom_broadcaster);
=======
    PublishOdom(odom_broadcaster);
>>>>>>> 8e0cd9742c3d0b6b9dfba2ddf925a0b7b33bc109
    PublishYaw();
    PublishGyro();
    PublishUltrasonic();
    ros::spinOnce();
    p_loop_rate->sleep();
  }
  return 0;
}

