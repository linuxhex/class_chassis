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
#include <sched.h>  // for setting priority
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include "wc_chassis_mcu.h"  // NOLINT
#if defined(VERIFY_REMTOE_ID)
#include "gs/file_util.h"
#endif

//#define SETTING_PRIORITY
#define VERIFY_REMOTE_KEY

WC_chassis_mcu g_chassis_mcu;

double ultral_effective_range = 0.4;
double g_odom_x   = 0;
double g_odom_y   = 0;
double g_odom_tha = 0;

double g_odom_v = 0;
double g_odom_w = 0;
std::vector<int> g_ultrasonic;
std::string ultrasonic;
/********************
  byte     0        1-13          14-17            18               19           20           21          22        23      24
       protector ultrasonic   reserve_ultra  hardware_status   device_status  battery_H    battery_L   charge_H   charge_L
 *******************/

unsigned int g_dio_count  = 0;
unsigned int g_ret_count  = 0;
unsigned int g_pc_control = 0;

double last_cmd_vel_time = 0.0;
double max_cmd_interval  = 1.0;
unsigned int connection_status = 1; // mcu ethernet connection status: 0>bad 1>good
double ACC_LIM_TH = 3.0 / 2.0 * M_PI;

float m_speed_v = 0.0;
float m_speed_w = 0.0;

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
int sum_battery_capacity = 0;
int display_battery_capacity = 50;
int battery_count = -1;
int battery_level_ = 3;

unsigned int remote_ret_ = 0x0a00;
unsigned int remote_ret_cnt_ = 0;
unsigned char remote_cmd_;
unsigned short remote_index_;

enum Device_ID{
  Emergency_stop = 0,
  Mode,
  Battery,
  Mileage,
  Device_MAX
};

std::string ultrasonic_str[15] = {"ultrasonic0","ultrasonic1","ultrasonic2","ultrasonic3","ultrasonic4",
                                  "ultrasonic5","ultrasonic6","ultrasonic7","ultrasonic8","ultrasonic9",
                                  "ultrasonic10","ultrasonic11","ultrasonic12","ultrasonic13","ultrasonic14"
                                  "ultrasonic15"};

ros::Publisher odom_pub;
ros::Publisher gyro_pub;
ros::Publisher remote_cmd_pub;
ros::Publisher going_back_pub;
ros::Publisher device_pub;
ros::Publisher rotate_finished_pub;
ros::Publisher yaw_pub;
ros::Publisher ultrasonic_pub[15];
ros::Publisher protector_pub;

ros::ServiceServer start_rotate_srv;
ros::ServiceServer stop_rotate_srv;
ros::ServiceServer check_rotate_srv;

void DoNavigation(const geometry_msgs::Twist& Navigation_msg) {
  timeval tv;
  gettimeofday(&tv, NULL);
  last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

  m_speed_v = Navigation_msg.linear.x;
  m_speed_w = Navigation_msg.angular.z;
//  ROS_INFO("Navigation.linear.x = %f, angular.z = %f", m_speed_v, m_speed_w);
}

void RemoteRetCallback(const std_msgs::UInt32& ret) {
  remote_ret_ = ret.data;
  remote_ret_cnt_ = 0;
  ROS_INFO("[wc_chassis] get remote ret cmd = %d, state = %d", (remote_ret_ & 0xff), (remote_ret_ >> 8) & 0xff);
}

void GyroUpdateCallback(const std_msgs::UInt32& state) {
  if (state.data == 0) {
    g_chassis_mcu.gyro_state_ = 0; 
  } else {
    g_chassis_mcu.gyro_state_ = 1; 
  }
  ROS_INFO("[wc_chassis] set gyro state = %d", g_chassis_mcu.gyro_state_);
}

void publish_protector_status() {
  std::bitset<32> status;
  std::string str;
  diagnostic_msgs::KeyValue value;
  status = g_ultrasonic[0];
  str = status.to_string();
  value.key = std::string("protector_data"); // 0:on 1:off
  value.value = str.substr(24, 8);
  protector_pub.publish(value);
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

  if((strcmp(frame_id,"ultrasonic8") == 0) || (strcmp(frame_id,"ultrasonic9") == 0)){
      dis_meter = dis_meter - 0.20;
  }

  if(dis_meter < range.min_range) {
    range.range = range.min_range;
  } else if (dis_meter > ultral_effective_range) {  // effective range
    range.range = range.max_range;
  } else {
    range.range = dis_meter;
  }
  publisher.publish(range);
}

void PublishUltrasonic() {
  for (int i=0;i<15;i++) {
    if ((ultrasonic.find(ultrasonic_str[i]) != std::string::npos) && (ultrasonic_pub[i] != 0)) {
      publish_ultrasonic(ultrasonic_pub[i], ultrasonic_str[i].c_str(), g_ultrasonic[1+i]);
    }
  }
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

  device_value.key = std::string("MCU_connection"); // 0:bad 1:good
  device_value.value = connection_status == 1 ? std::string("true") : std::string("false");
  device_status.values.push_back(device_value);

  unsigned int battery_ADC = (g_ultrasonic[20] << 8) | (g_ultrasonic[21] & 0xff);
//  double battery_value = 0.2393 * battery_ADC - 125.04;
//  double battery_value = 0.22791 * (battery_ADC - 516);
  double battery_value = 0.2298 * (battery_ADC - 516);
  int current_battery_capacity;
	current_battery_capacity = (battery_value - battery_empty_level) / (battery_full_level - battery_empty_level) * 100;
  if(current_battery_capacity < 0) current_battery_capacity = 0; 
  if(current_battery_capacity > 100) current_battery_capacity = 100; 
  ++battery_count;
  if(battery_count == 0) {
    display_battery_capacity = current_battery_capacity;
  } else if(battery_count > 0) {
    sum_battery_capacity += current_battery_capacity;
    if(battery_count == 300) {
      display_battery_capacity = sum_battery_capacity / 300;
      battery_count = 0;
      sum_battery_capacity = 0;
    }
  }
  if (display_battery_capacity < 10) {
    battery_level_ = 0;
  } else if (display_battery_capacity < 40) {
    battery_level_ = 1;
  } else if (display_battery_capacity < 75) {
    battery_level_ = 2;
  } else {
    battery_level_ = 3;
  }

  std::cout << "battery_ADC " << battery_ADC << "; battery_balue " << battery_value << "; current_battery_capacity " << current_battery_capacity << "; display_battery_capacity " << display_battery_capacity << " battery_level_" << battery_level_ << std::endl;
  device_value.key = std::string("battery");
  device_value.value = std::to_string(display_battery_capacity);
  device_status.values.push_back(device_value);

  device_value.key = std::string("mileage");
  unsigned int mileage = (unsigned int)((g_chassis_mcu.mileage_left_ + g_chassis_mcu.mileage_left_) / 2);
  device_value.value = std::to_string(mileage);
  device_status.values.push_back(device_value);

  device_pub.publish(device_status);
}

void PublishRotateFinished(int is_finished) {
  std_msgs::Int32 msg;
  msg.data = is_finished;
  rotate_finished_pub.publish(msg);
}

bool DoRotate() {
  if (fabs(g_chassis_mcu.acc_odom_theta_) >= fabs(rotate_angle / 180.0 * M_PI) ) {
    start_rotate_flag = false;
    is_rotate_finished = true;
    g_chassis_mcu.setTwoWheelSpeed(0.0, 0.0);
//    ROS_INFO("[wc_chassis] rotate finished!");
    PublishRotateFinished(1);
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
      PublishRotateFinished(1);
    }
  }
  return true;
}

void DoDIO() {
  if ((++g_dio_count % 2) == 0) {
    unsigned int temp_di_data = g_chassis_mcu.doDIO(g_do_data_);
    cur_emergency_status = (temp_di_data >> (8 + Emergency_stop)) & 0x01;
    if (g_dio_count > 2 && ((g_di_data_ & 0x03) != 0x03) && ((temp_di_data & 0x03) == 0x03)) {
      ROS_INFO("[wc_chassis] get_di data: 0x%x, and then publish going back!!!", temp_di_data);
      std_msgs::UInt32 msg;
      msg.data = 1;
      going_back_pub.publish(msg); 
    }
    g_di_data_ = temp_di_data;
    g_do_data_ = g_di_data_; 
  }
}

void DoRemoteRet() {
  if (++remote_ret_cnt_ > 8) {
    remote_ret_ &= 0xff00;
  }
  g_chassis_mcu.setRemoteRet(remote_ret_);
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

#ifdef VERIFY_REMOTE_KEY 
unsigned int GenerateJSHash(unsigned int seed) {
  unsigned char ch[] = "NTM4N2I2YmFiOWIwNzgzYmViYWFjYjc2"; 
  unsigned int hash = seed;
  for(int i = 0; i < 32; i++) {
    hash ^= ((hash << 5) + ch[i] + (hash >> 2));
  }
  for(int i = 0; i < 32; i++) {
    hash ^= (((hash - ch[i]) >> 2) + (hash << 3));
  }
  return hash;
}
#endif

int main(int argc, char **argv) {
  ros::init(argc, argv, "wc_chassis");
#ifdef SETTING_PRIORITY
  struct sched_param param;
  param.sched_priority = 99;
  if (0 != sched_setscheduler(getpid(), SCHED_RR, &param)) {
    std::cout << "set priority failed" << std::endl;
  } else {
    std::cout << "set priority succeed" << std::endl;
  }
#endif
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  ros::NodeHandle device_nh("device");

  tf::TransformBroadcaster odom_broadcaster;

  double f_dia = 0;
  double b_dia = 0;
  double axle  = 0;
  int counts = 0;
  int reduction_ratio = 0;
  int remote_id = 1;
  double speed_ratio = 1.0;
  double timeWidth = 0;
  double max_speed_v, max_speed_w; 
  double speed_v_acc, speed_v_dec, speed_v_dec_zero;
  double speed_w_acc, speed_w_dec;
  double full_speed;
  int delta_counts_th;
  std::string host_name;
  int port;
  std::string str_odom = "odom";

//  nh.param("remote_id", remote_id, 1);
  nh.param("host_name", host_name, std::string("10.7.5.199"));
  nh.param("port", port, 5000);
  nh.param("odom", str_odom, str_odom);
  nh.param("max_cmd_interval", max_cmd_interval, 1.0);
  nh.param("F_DIA", f_dia, static_cast<double>(0.125));	// diameter of front wheel
  nh.param("B_DIA", b_dia, static_cast<double>(0.125));
  nh.param("AXLE", axle, static_cast<double>(0.383));		// length bettween two wheels
  nh.param("COUNTS", counts, 12);
  nh.param("REDUCTION_RATIO", reduction_ratio, 30);
  nh.param("SPEED_RATIO", speed_ratio, static_cast<double>(1.0));
  nh.param("TimeWidth", timeWidth, static_cast<double>(0.1));

  nh.param("max_speed_v", max_speed_v, static_cast<double>(0.6));
  nh.param("max_speed_w", max_speed_w, static_cast<double>(0.6));
  nh.param("speed_v_acc", speed_v_acc, static_cast<double>(0.025));
  nh.param("speed_v_dec", speed_v_dec, static_cast<double>(-0.12));
  nh.param("speed_v_dec_to_zero", speed_v_dec_zero, static_cast<double>(-0.12));
  nh.param("speed_w_acc", speed_w_acc, static_cast<double>(0.25));
  nh.param("speed_w_dec", speed_w_dec, static_cast<double>(-0.25));
  nh.param("full_speed",full_speed,static_cast<double>(3.0));
  nh.param("delta_counts_th",delta_counts_th,40);

  nh.param("ultral_effective_range", ultral_effective_range, static_cast<double>(0.4));
  nh.param("battery_full_level", battery_full_level, static_cast<double>(27.5));
  nh.param("battery_empty_level", battery_empty_level, static_cast<double>(20.0));
  nh.param("ultrasonic",ultrasonic,std::string(" "));
  std::cout << "F_DIA:" << f_dia << " B_DIA:" << b_dia << " AXLE:" << axle << " reduction_ratio: " << reduction_ratio << " speed_ratio:" << speed_ratio << std::endl;
  std::cout << "max_speed_v:" << max_speed_v << " max_speed_w:" << max_speed_w << " speed_v_acc:" << speed_v_acc << " speed_v_dec: " << speed_v_dec << " speed_w_dec_to_zero:" << speed_v_dec_zero << " speed_w_acc:" << speed_w_acc  << " speed_w_dec:" << speed_w_dec << std::endl;

//  yaw_pub = n.advertise<std_msgs::Float32>("yaw", 10);
  odom_pub  = n.advertise<nav_msgs::Odometry>("odom", 50);
  gyro_pub  = device_nh.advertise<sensor_msgs::Imu>("gyro", 50);
  remote_cmd_pub  = device_nh.advertise<std_msgs::UInt32>("remote_cmd", 50);
  going_back_pub  = device_nh.advertise<std_msgs::UInt32>("cmd_going_back", 50);
  rotate_finished_pub = device_nh.advertise<std_msgs::Int32>("rotate_finished", 5);
  device_pub = device_nh.advertise<diagnostic_msgs::DiagnosticStatus>("device_status", 50);
  protector_pub = device_nh.advertise<diagnostic_msgs::KeyValue>("protector", 50);

  for (int i = 0; i < 15; i++) {
    if (ultrasonic.find(ultrasonic_str[i]) != std::string::npos) {
      ultrasonic_pub[i] = n.advertise<sensor_msgs::Range>(ultrasonic_str[i].c_str(), 50);
    }
  }

  start_rotate_srv = device_nh.advertiseService("start_rotate", &StartRotate);
  stop_rotate_srv = device_nh.advertiseService("stop_rotate", &StopRotate);
  check_rotate_srv = device_nh.advertiseService("check_rotate", &CheckRotate);

  ros::Subscriber Navi_sub = n.subscribe("cmd_vel", 10, DoNavigation);
  ros::Subscriber remote_ret_sub = n.subscribe("/device/remote_ret", 10, RemoteRetCallback);
  ros::Subscriber gyro_update_state_sub = n.subscribe("/gyro_update_state", 10, GyroUpdateCallback);

#if defined(VERIFY_REMTOE_ID)
  std::string str_id;
  if (!gs::file::ReadFile("param_device", str_id)) {
    remote_id = 2;
  } else {
    str_id = str_id.substr(str_id.find(' ') + 1, str_id.size());
	  int temp_id = std::atoi(str_id.c_str());
    remote_id = temp_id > 0 && temp_id < 10 ? temp_id : 1;
    std::cout << "remote_id = " << remote_id << std::endl;
  }
#endif

  ROS_INFO("waiting network w5500 start....");
//  sleep(10);
  g_chassis_mcu.Init(host_name, std::to_string(port), 0.975, f_dia, b_dia, axle, timeWidth, counts, reduction_ratio, speed_ratio, 
                     max_speed_v, max_speed_w, speed_v_acc, speed_v_dec, speed_v_dec_zero, speed_w_acc, speed_w_dec,full_speed,delta_counts_th);

  sleep(1);

#ifdef VERIFY_REMOTE_KEY 
  srand((unsigned int)time(NULL));
  unsigned int seed_key = rand();
  unsigned int check_key = GenerateJSHash(seed_key);
  unsigned int verify_key = g_chassis_mcu.checkRemoteVerifyKey(seed_key);
//  std::cout << "seed_key = " << seed_key << "; generate hash check key = " << check_key << "; verify_key = " << verify_key << std::endl;
  if (check_key != verify_key) {
//    std::cout << "check key failed, return directly" << std::endl;
    exit(0);
  }
#endif
  g_chassis_mcu.setRemoteID((unsigned char)((remote_id & 0x3f) | (battery_level_ << 6)));
//  std::cout << "Start Main Loop!" << std::endl;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    g_chassis_mcu.getOdo(g_odom_x, g_odom_y, g_odom_tha);
    g_chassis_mcu.getCSpeed(g_odom_v, g_odom_w);
    usleep(1000);
    timeval tv;
    gettimeofday(&tv, NULL);
    double time_now = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

    if(start_rotate_flag) {
      DoRotate();
    } else {
      if (time_now - last_cmd_vel_time >= max_cmd_interval) {
        g_chassis_mcu.setTwoWheelSpeed(0.0,0.0);
      } else {
        g_chassis_mcu.setTwoWheelSpeed(m_speed_v, m_speed_w);
      }
    }
   // set do get di
    DoDIO();
    DoRemoteRet();
    if (++loop_count % 2) {
      g_chassis_mcu.getRemoteCmd(remote_cmd_, remote_index_);
      PublisheRemoteCmd(remote_cmd_, remote_index_); 
      publish_device_status();
    }
    if (loop_count % 10) {
      g_chassis_mcu.setRemoteID((unsigned char)((remote_id & 0x3f) | (battery_level_ << 6)));
      loop_count = 0;
    }
    //发布里程计
    PublishOdom(&odom_broadcaster);
//    PublishYaw();
    PublishGyro();
    // publish ultrasonic data
    PublishUltrasonic();
    // publish protector status
    publish_protector_status();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
