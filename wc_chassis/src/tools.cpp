#include "tools.h"
unsigned int remote_ret_ = 0x0a00;
unsigned int remote_ret_cnt_ = 0;

void RemoteRetCallback(const std_msgs::UInt32& ret) {
  remote_ret_ = ret.data;
  remote_ret_cnt_ = 0;
  ROS_INFO("[wc_chassis] get remote ret cmd = %d, state = %d", (remote_ret_ & 0xff), (remote_ret_ >> 8) & 0xff);
}

void DoRemoteRet(WC_chassis_mcu& g_chassis_mcu) {
  if (++remote_ret_cnt_ > 15) {
    remote_ret_ &= 0xff00;
  }
  g_chassis_mcu.setRemoteRet(remote_ret_);
}

