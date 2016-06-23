/* action.cpp 所有的设备的控制动作逻辑
 */
#include "action.h"
#include "init.h"
#include "parameter.h"
#include <std_msgs/Int32.h>

bool DoRotate(ros::Publisher &rotate_finished_pub) {
  if (fabs(g_chassis_mcu->acc_odom_theta_) >= fabs(rotate_angle / 180.0 * M_PI * 0.98) ) {
    start_rotate_flag = false;
    is_rotate_finished = true;
    g_chassis_mcu->setTwoWheelSpeed(0.0, 0.0);
    std_msgs::Int32 msg;
    msg.data = 1;
    rotate_finished_pub.publish(msg);

    timeval tv;
    gettimeofday(&tv, NULL);
    last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
  } else {
    is_rotate_finished = false;
    if (rotate_angle > 0) {
      g_chassis_mcu->setTwoWheelSpeed(0.0, 0.2);
    } else if (rotate_angle < 0) {
      g_chassis_mcu->setTwoWheelSpeed(0.0, -0.2);
    } else {
      is_rotate_finished = true;
      start_rotate_flag = false;
      std_msgs::Int32 msg;
      msg.data = 1;
      rotate_finished_pub.publish(msg);
      g_chassis_mcu->setTwoWheelSpeed(0.0, 0.0);
    }
  }
  return true;
}

void DoDIO(ros::Publisher going_back_pub) {
    if ((++g_dio_count % 2) == 0) {
      unsigned int temp_di_data = g_chassis_mcu->doDIO(g_do_data_);
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

void DoRemoteRet(void) {
  if (++remote_ret_cnt_ > 8) {
    remote_ret_ &= 0xff00;
  }
  g_chassis_mcu->setRemoteRet(remote_ret_);
}
