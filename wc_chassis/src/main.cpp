/* Copyright(C) Gaussian Robot. All rights reserved.
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <autoscrubber_services/LaunchScrubber.h>
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

WC_chassis_mcu g_chassis_mcu;

double ACC_LIM_TH = 3.0 / 2.0 * M_PI;
double ultral_effective_range = 0.4;
double g_odom_x   = 0;
double g_odom_y   = 0;
double g_odom_tha = 0;

double g_odom_v = 0;
double g_odom_w = 0;
std::vector<int> g_ultrasonic;

double f_zero_;     // 零度角偏移
double f_speed_k_;  // 速递k系数

U32 g_plc_auto  = 1;   // 默认自动模式
U32 g_plc_front = 0;   // 默认中间模式

unsigned int g_dio_count  = 0;
unsigned int g_ret_count  = 0;
unsigned int g_pc_control = 0;

double last_cmd_vel_time = 0.0;
double max_cmd_interval  = 1.0;

bool scrub_need_stop          = true;
bool launch_scrubber          = false;
double v_scrub_threshold      = 0.07;
double radius_scrub_threshold = 1.0;

int current_v_index = 0;
int current_w_index = 0;
float m_speed_v = 0.0;
float m_speed_w = 0.0;
float g_speed_v[3] = {0.0, 0.0, 0.0};
float g_speed_w[3] = {0.0, 0.0, 0.0};
pthread_mutex_t speed_mutex;
float g_spe = 0.0;
float g_angle = 0.0;
int g_planner_type = 0;
unsigned int loop_count = 0;
unsigned int rotate_angle = 0;

bool start_rotate_flag = false;
bool stop_rotate_flag = true;
bool is_rotate_finished = false;

unsigned int remote_ret_ = 0x0a00;
unsigned int remote_ret_cnt_ = 0;
unsigned char remote_cmd_;
unsigned short remote_index_;

enum RemoteCmdType {
  TYPE_CANCEL = -1,
  TYPE_STOP = 0,
  TYPE_START = 1
};

enum RemoteCmd {
  CMD_NULL = 0,
  CMD_SET_GOAL, 
  CMD_INIT,
  CMD_SCAN_MAP,
  CMD_RECORD_PATH,
  CMD_MARK_GOAL,
  CMD_GUIDE,
  CMD_USHER,
  CMD_PAUSE,
  CMD_RESUME,
  CMD_TERMINATE,
  CMD_MOVE,
  CMD_RETURN,
  CMD_MAX 
};

enum RemoteRet {
  RET_NULL = 0,
  RET_INFO, 
  RET_CONFORM,
  RET_ERROR,
  RET_WARN
};

enum ErrorIndex {
  E_NULL = 0,
  E_INIT_FAILED,
  E_PATH_NOT_SAFE,
  E_LOCATION_INVALID,
  E_PLANNING_INVALID,
  E_FRONT_NOT_SAFE,
  E_FOOTPRINT_NOT_SAFE,
  E_GOAL_UNREACHED 
};

enum WarnIndex {
  W_NULL = 0,
  W_MAP_NOT_EXSIT,
  W_MAP_EXSIT,
  W_PATH_NOT_EXSIT,
  W_PATH_EXSIT,
  W_GOAL_EXSIT

};

enum InfoIndex {
  I_NULL = 0,
  I_SLAM_FINISHED,
  I_RECORDPATH_FINISHED,
  I_MARKGOAL_FINISHED,
  I_INIT_FINISHED,
  I_GOAL_REACHED

};

ros::Publisher odom_pub;
ros::Publisher gyro_pub;
ros::Publisher remote_cmd_pub;
ros::Publisher model_pub;
ros::Publisher yaw_pub;
ros::Publisher mark_goal_pub;
ros::Publisher set_goal_pub;
ros::Publisher mark_init_pub;
ros::Publisher set_init_pub;
ros::Publisher pause_pub;
ros::Publisher terminate_pub;
ros::Publisher scan_map_pub;
ros::Publisher record_path_pub;
ros::Publisher ultrasonic0_pub;
ros::Publisher ultrasonic1_pub;
ros::Publisher ultrasonic2_pub;
ros::Publisher ultrasonic3_pub;
ros::Publisher ultrasonic4_pub;
ros::Publisher ultrasonic5_pub;
ros::ServiceServer launch_scrubber_srv;
ros::ServiceServer stop_scrubber_srv;

ros::ServiceServer start_rotate_srv;
ros::ServiceServer stop_rotate_srv;
ros::ServiceServer check_rotate_srv;

// Data Output
#define DO_SUCK_ENABLE 0X0080     // 0 -> not enable, 1 -> enable
#define DO_BRUSH_ENABLE 0X0040    // 0 -> not enable, 1 -> enable
#define DO_SUCK_UP 0X8000         // 01 -> down, 10 -> up
#define DO_SUCK_DOWN 0x4000       // as above
#define DO_BRUSH_UP 0X2000        // 0 -> up, 1 -> down
#define DO_BRUSH_DOWN 0X1000      // as above

// Data Input
#define DI_FORWARD 0X0001
#define DI_BACKWARD 0X0002
#define DI_BRUSH_UP_DOWN 0X0004
#define DI_BRUSH_ENABLE 0X0008
#define DI_GAUGE1 0X0010
#define DI_GAUGE2 0X0020
#define DI_SUCK_UP_DOWN 0X0100
#define DI_SUCK_ENABLE 0X0200
#define DI_MODE 0X0400
#define DI_ENMERGENCY_STOP 0X2000

//#define SETTING_PRIORITY

const int g_scrub_mode = DO_SUCK_ENABLE | DO_BRUSH_ENABLE | DO_SUCK_DOWN | DO_BRUSH_DOWN | (255 << 16);
const int g_silent_mode = DO_SUCK_UP | DO_BRUSH_UP;
const int g_scrub_without_water_mode = DO_SUCK_ENABLE | DO_BRUSH_ENABLE | DO_SUCK_DOWN | DO_BRUSH_DOWN;

void PlanTypeCallBack(const std_msgs::UInt32& msg) {
  g_planner_type = msg.data;
}

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

  if (temp_v <= v_scrub_threshold || (temp_w != 0 && temp_v / fabs(temp_w) <= radius_scrub_threshold)) {
    scrub_need_stop = true;
  } else {
    scrub_need_stop = false;
  }
}

void WC_UD_CallBack(const std_msgs::UInt32& up_down) {
  g_pc_control = up_down.data;
}

void RemoteRetCallback(const std_msgs::UInt32& ret) {
  remote_ret_ = ret.data;
  if (remote_ret_ & 0xff != 0) {
    remote_ret_cnt_ = 0;
  } 
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
void PublishYaw(){
  std_msgs::Float32 msg;
  msg.data = g_odom_tha * 180.0 / M_PI;
  yaw_pub.publish(msg);
}
void PublishModel() {
  if (g_chassis_mcu.is_Auto()) {
    std_msgs::UInt32 msg;
    msg.data = 1;
    model_pub.publish(msg);
  } else {
    std_msgs::UInt32 msg;
    msg.data = 0;
    model_pub.publish(msg);
  }
}
/*
void PublishMarkGoal(unsigned int mark_index) {
  //ROS_INFO("[wc_chassis] publish mark goal index = %d", mark_index);
  std_msgs::UInt32 msg;
  msg.data = mark_index; 
  mark_goal_pub.publish(msg);
}

void PublishSetGoal(unsigned int set_index) {
  //ROS_INFO("[wc_chassis] publish set goal index = %d", set_index);
  std_msgs::UInt32 msg;
  msg.data = set_index; 
  set_goal_pub.publish(msg);
}

void PublishGoingBack() {
  PublishSetGoal(0);
}

void PublishMarkInit(unsigned int mark_index) {
  //ROS_INFO("[wc_chassis] publish mark init index = %d", mark_index);
  std_msgs::UInt32 msg;
  msg.data = mark_index; 
  mark_init_pub.publish(msg);
}

void PublishSetInit(unsigned int set_index) {
  //ROS_INFO("[wc_chassis] publish set init index = %d", set_index);
  std_msgs::UInt32 msg;
  msg.data = set_index; 
  set_init_pub.publish(msg);
}

void PublishPause() {
  //ROS_INFO("[wc_chassis] request pause action");
  std_msgs::UInt32 cmd_pause;
  cmd_pause.data = 1;
  pause_pub.publish(cmd_pause);
}

void PublishResume() {
  //ROS_INFO("[wc_chassis] request resume action");
  std_msgs::UInt32 cmd_resume;
  cmd_resume.data = 0;
  pause_pub.publish(cmd_resume);
}

void PublishTerminate() {
  //ROS_INFO("[wc_chassis] request terminate action");
  std_msgs::UInt32 cmd_terminate;
  cmd_terminate.data = 1;
  terminate_pub.publish(cmd_terminate);
}

void PublishScanMap(int cmd_type) {
  std_msgs::Int8 cmd_scan_map;
  cmd_scan_map.data = cmd_type;
  scan_map_pub.publish(cmd_scan_map);
}

void PublishRecordPath(int cmd_type) {
  std_msgs::Int8 cmd_record_path;
  cmd_record_path.data = cmd_type;
  record_path_pub.publish(cmd_record_path);
}
*/

void PublisheRemoteCmd(unsigned char cmd, unsigned short index) {
  std_msgs::UInt32 remote_cmd; 
  if (cmd > 0 && cmd < 12) {
    ROS_INFO("[wc_chassis] get remote cmd = %d, index = %d", cmd, index);
  }
  remote_cmd.data = (index << 8) | cmd; 
  remote_cmd_pub.publish(remote_cmd);
}

bool DoRotate() {
  if(start_rotate_flag) {
    if (fabs(g_chassis_mcu.acc_odom_theta_) >= fabs(rotate_angle / 180.0 * M_PI * 0.98) ) {
      start_rotate_flag = false;
      is_rotate_finished = true;
      g_chassis_mcu.setTwoWheelSpeed(0.0, 0.0);
//      ROS_INFO("[wc_chassis] rotate finished!");
      timeval tv;
      gettimeofday(&tv, NULL);
      last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
    } else {
      is_rotate_finished = false;
//      ROS_INFO("[wc_chassis] inplace rotation: cur_yaw = %lf, targer_yaw = %d ", g_chassis_mcu.acc_odom_theta_ * 57.3, rotate_angle);
      if (rotate_angle > 0) {
        g_chassis_mcu.setTwoWheelSpeed(0.0, 0.2);
//        ROS_INFO("[wc_chassis] rotate to left");
      } else if (rotate_angle < 0) {
        g_chassis_mcu.setTwoWheelSpeed(0.0, -0.2);
//        ROS_INFO("[wc_chassis] rotate to right");
      } else {
        is_rotate_finished = true;
        start_rotate_flag = false;
        g_chassis_mcu.setTwoWheelSpeed(0.0, 0.0);
      }
    }
    return true;
  } else {
    return false;		
  }
		
}

void DoDIO() {
  if ((g_dio_count++ % 2) == 0) {
    g_chassis_mcu.setDO(remote_ret_);
  }
}

void DoRemoteRet() {
  if (++remote_ret_cnt_ > 15) {
    remote_ret_ &= 0xff00;
  }
  g_chassis_mcu.setRemoteRet(remote_ret_);
}

bool LaunchScrubber(autoscrubber_services::LaunchScrubber::Request& req, autoscrubber_services::LaunchScrubber::Response& res) {
  launch_scrubber = true;
  ROS_INFO("Launch Scrubber!");
  return true;
}

bool StopScrubber(autoscrubber_services::StopScrubber::Request& req, autoscrubber_services::StopScrubber::Response& res) {
  launch_scrubber = false;
  return true;
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
  double speed_ratio = 1.0;
  double timeWidth = 0;
  std::string host_name;
  int port;
  std::string str_auto_topic;
  std::string str_odom = "odom";

  nh.param("angle_zero", f_zero_, 0.0);
  nh.param("speed_k", f_speed_k_, 1.0);
  nh.param("odom", str_odom, str_odom);
  nh.param("WC_Auto_topic", str_auto_topic, std::string("WC_AUTO"));
  nh.param("max_cmd_interval", max_cmd_interval, 1.0);
  nh.param("v_scrub_threshold", v_scrub_threshold, 0.07);
  nh.param("radius_scrub_threshold", radius_scrub_threshold, 1.0);
  nh.param("F_DIA", f_dia, static_cast<double>(0.125));	// diameter of front wheel
  nh.param("B_DIA", b_dia, static_cast<double>(0.125));
  nh.param("AXLE", axle, static_cast<double>(0.383));		// length bettween two wheels
  nh.param("COUNTS", counts, 12);
  nh.param("REDUCTION_RATIO", reduction_ratio, 30);
  nh.param("SPEED_RATIO", speed_ratio, static_cast<double>(1.0));
  nh.param("TimeWidth", timeWidth, static_cast<double>(0.1));
  nh.param("ultral_effective_range", ultral_effective_range, static_cast<double>(0.4));
  nh.param("host_name", host_name, std::string("192.168.1.199"));
  nh.param("port", port, 5000);
  nh.param("acc_lim_th", ACC_LIM_TH, 3.0 / 2.0 * M_PI);
  std::cout << "F_DIA:" << f_dia << " B_DIA:" << b_dia << " AXLE:" << axle << " reduction_ratio: " << reduction_ratio << " speed_ratio:" << speed_ratio << std::endl;
  std::cout << "v_scrub: " << v_scrub_threshold << " radius_scrub: " << radius_scrub_threshold << std::endl;

  model_pub = n.advertise<std_msgs::UInt32>(str_auto_topic, 10);
/*
  mark_goal_pub = n.advertise<std_msgs::UInt32>("mark_goal", 10); 
  set_goal_pub = n.advertise<std_msgs::UInt32>("set_goal", 10); 
  mark_init_pub = n.advertise<std_msgs::UInt32>("mark_init", 10); 
  set_init_pub = n.advertise<std_msgs::UInt32>("set_init", 10); 
  pause_pub = n.advertise<std_msgs::UInt32>("/move_base_simple/gaussian_pause", 10); 
  terminate_pub = n.advertise<std_msgs::UInt32>("/move_base_simple/gaussian_cancel", 10); 
  scan_map_pub = n.advertise<std_msgs::Int8>("/remote_scan_map", 10); 
  record_path_pub = n.advertise<std_msgs::Int8>("/remote_record_path", 10); 
*/

  yaw_pub = n.advertise<std_msgs::Float32>("yaw", 10);
  odom_pub  = n.advertise<nav_msgs::Odometry>("odom", 50);
  gyro_pub  = n.advertise<sensor_msgs::Imu>("gyro", 50);
  remote_cmd_pub  = n.advertise<std_msgs::UInt32>("/device/remote_cmd", 50);
  ultrasonic0_pub = n.advertise<sensor_msgs::Range>("ultrasonic0", 50);
  ultrasonic1_pub = n.advertise<sensor_msgs::Range>("ultrasonic1", 50);
  ultrasonic2_pub = n.advertise<sensor_msgs::Range>("ultrasonic2", 50);
  ultrasonic3_pub = n.advertise<sensor_msgs::Range>("ultrasonic3", 50);
  ultrasonic4_pub = n.advertise<sensor_msgs::Range>("ultrasonic4", 50);
  ultrasonic5_pub = n.advertise<sensor_msgs::Range>("ultrasonic5", 50);

  launch_scrubber_srv = n.advertiseService("launch_scrubber", &LaunchScrubber);
  stop_scrubber_srv = n.advertiseService("stop_scrubber", &StopScrubber);
  start_rotate_srv = device_nh.advertiseService("start_rotate", &StartRotate);
  stop_rotate_srv = device_nh.advertiseService("stop_rotate", &StopRotate);
  check_rotate_srv = device_nh.advertiseService("check_rotate", &CheckRotate);

  ros::Subscriber Navi_sub   = n.subscribe("cmd_vel", 10, DoNavigation);
  ros::Subscriber sb_Up_Down = n.subscribe("WC_UP_DOWN", 10, WC_UD_CallBack);
  ros::Subscriber planner_type_sub = n.subscribe("planner_type", 10, PlanTypeCallBack);
  ros::Subscriber remote_ret_sub = n.subscribe("/device/remote_ret", 10, RemoteRetCallback);
  ros::Subscriber gyro_update_state_sub = n.subscribe("/gyro_update_state", 10, GyroUpdateCallback);

  pthread_mutex_init(&speed_mutex, NULL);

  g_chassis_mcu.Init(host_name, std::to_string(port), 0.975, f_dia, b_dia, axle, timeWidth, counts, reduction_ratio, speed_ratio);
  g_chassis_mcu.setThaZero(f_zero_);

//  std::cout << "Start Main Loop!" << std::endl;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    g_chassis_mcu.getOdo(g_odom_x, g_odom_y, g_odom_tha);
    g_chassis_mcu.getCSpeed(g_odom_v, g_odom_w);
    usleep(1000);
    timeval tv;
    gettimeofday(&tv, NULL);
    double time_now = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
    if (!DoRotate()) {
      if (time_now - last_cmd_vel_time >= max_cmd_interval) {
//     g_chassis_mcu.setSpeed(0.0, 0.0, 1);
        g_chassis_mcu.setTwoWheelSpeed(0.0,0.0);
        scrub_need_stop = true;
      } else {
        pthread_mutex_lock(&speed_mutex);
        float speed_v = g_speed_v[current_v_index];
        float speed_w = g_speed_w[current_w_index];
        pthread_mutex_unlock(&speed_mutex);
//      g_chassis_mcu.setSpeed(speed_v, speed_w, g_planner_type);
        g_chassis_mcu.setTwoWheelSpeed(m_speed_v, m_speed_w);
      }
    }
   // set do get di
//    DoDIO();
    DoRemoteRet();
    if (++loop_count % 2) {
      g_chassis_mcu.getRemoteCmd(remote_cmd_, remote_index_);
      PublisheRemoteCmd(remote_cmd_, remote_index_); 
      loop_count = 0;
    }
    //发布里程计
    PublishOdom(&odom_broadcaster);
    //发布模式状态
    PublishModel();
    PublishYaw();
    PublishGyro();
    // publish ultrasonic data
    PublishUltrasonic();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

