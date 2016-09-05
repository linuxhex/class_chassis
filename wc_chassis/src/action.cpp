/* action.cpp 所有的设备的控制动作逻辑
 */
#include "action.h"
#include "init.h"
#include "parameter.h"
#include <std_msgs/Int32.h>
/*
 * 初始化　旋转操作
 */
bool DoRotate(ros::Publisher &rotate_finished_pub) {
   if (fabs(p_chassis_mcu->acc_odom_theta_) >= fabs(rotate_angle / 180.0 * M_PI * 0.98) || rotate_angle == 0) {
    start_rotate_flag = false;
    is_rotate_finished = true;
    p_chassis_mcu->setTwoWheelSpeed(0.0, 0.0);
    std_msgs::Int32 msg;
    msg.data = 1;
    rotate_finished_pub.publish(msg);

    timeval tv;
    gettimeofday(&tv, NULL);
    last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
  } else {
       is_rotate_finished = false;
       int remain_angle = fabs(rotate_angle) - fabs(p_chassis_mcu->acc_odom_theta_) / M_PI *180;
       if(remain_angle < 20){
         double speed_w = remain_angle / 20.0 * p_speed_w->inplace_rotating_theta;
         speed_w = (speed_w < 0.10) ? 0.10 : speed_w;
         if (rotate_angle > 0) {
           p_chassis_mcu->setTwoWheelSpeed(0.0, speed_w);
         }else{
           p_chassis_mcu->setTwoWheelSpeed(0.0, -speed_w);
         }
       } else {
         if(rotate_angle > 0) {
           p_chassis_mcu->setTwoWheelSpeed(0.0, p_speed_w->inplace_rotating_theta);
         } else {
           p_chassis_mcu->setTwoWheelSpeed(0.0, -1 * p_speed_w->inplace_rotating_theta);
         }
       }
  }
  return true;
}

void onCharge(void)
{
    if(sleep_cnt == 0){
        charger_cmd_ = CMD_CHARGER_ON;
        std::thread([&](){
          GS_INFO("[CHASSIS] start to check charger volatage = %d", charger_voltage_);
          sleep(3);
          int check_charger_cnt = 0;
          while(++sleep_cnt  < 60 && charger_cmd_ == CMD_CHARGER_ON) {
           GS_INFO("[CHASSIS] checking charger volatage = %d", charger_voltage_);
           if (charger_voltage_ >= charger_low_voltage_) {
             ++check_charger_cnt;
           } else {
             check_charger_cnt = 0;
           }
           if (check_charger_cnt > charger_delay_time_ && charger_cmd_ == CMD_CHARGER_ON) {
             GS_INFO("[CHASSIS] check charger voltage normal > 30s, set charger relay on!!!");
             p_chassis_mcu->setChargeCmd(CMD_CHARGER_ON);
             pre_mileage = (p_chassis_mcu->mileage_right_ + p_chassis_mcu->mileage_left_) / 2;
             break;
           }
            sleep(1);
          }
          sleep_cnt = 0; //允许下次线程能够进入
        }).detach();
    }
}

/*
 * IO口控制，手触等
 */
void DoDIO(ros::Publisher going_back_pub) {
  unsigned int temp_di_data = p_chassis_mcu->doDIO(g_do_data_);
  GS_INFO("[wc_chassis] get_di data: 0x%x", temp_di_data);
  cur_emergency_status = (temp_di_data >> (8 + Emergency_stop)) & 0x01;
  // new version hardware hand touch
  if (new_hand_touch_) {
    temp_di_data = g_ultrasonic[0];
  }
  if (++g_dio_count > 2 && ((g_di_data_ & 0x03) != 0x03) && ((temp_di_data & 0x03) == 0x03)) {
    GS_INFO("[wc_chassis] get_di data: 0x%x, and then publish going back!!!", temp_di_data);
    std_msgs::UInt32 msg;
    msg.data = 1;
    going_back_pub.publish(msg);
  }
  g_di_data_ = temp_di_data & 0xff;
  g_do_data_ = g_di_data_;  // just for testing
}
/*
 * 遥控器状态
 */
void DoRemoteRet(void) {
  if (++remote_ret_cnt_ > 8) {
    remote_ret_ &= 0xff00;
  }
  p_chassis_mcu->setRemoteRet(remote_ret_);
}
