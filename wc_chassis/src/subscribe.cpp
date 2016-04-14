#include"subscribe.h"
#include"init.h"
#include "parameter.h"
void DoNavigationCallback(const geometry_msgs::Twist& Navigation_msg) {
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

void RemoteRetCallback(const std_msgs::UInt32& ret) {
  remote_ret_ = ret.data;
  if (remote_ret_ & 0xff != 0) {
    remote_ret_cnt_ = 0;
  }
  ROS_INFO("[wc_chassis] get remote ret cmd = %d, state = %d", (remote_ret_ & 0xff), (remote_ret_ >> 8) & 0xff);
}

void GyroUpdateCallback(const std_msgs::UInt32& state) {
  if (state.data == 0) {
    g_chassis_mcu->gyro_state_ = 0;
  } else {
    g_chassis_mcu->gyro_state_ = 1;
  }
  ROS_INFO("[wc_chassis] set gyro state = %d", g_chassis_mcu->gyro_state_);
}

