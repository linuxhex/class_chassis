/* action.cpp 所有的设备的控制动作逻辑
 */
#include "action.h"
#include "init.h"
#include "parameter.h"
#include <std_msgs/Int32.h>


/*
 * 初始化　旋转操作
 */
bool DoRotate(void) {
       static double yaw_offset = 0.10;
       double target_angle = fabs(rotate_angle / 180.0 * M_PI);
       double target_yaw = target_angle < 1.2 * yaw_offset ? (target_angle + yaw_offset) : target_angle;

       double current_yaw = p_chassis_mcu->acc_odom_theta_;
       double rotate_yaw = fabs(current_yaw - pre_yaw);
       sum_yaw += rotate_yaw > M_PI ? (2.0 * M_PI - rotate_yaw) : rotate_yaw;
       double rest_yaw = target_yaw - sum_yaw;
       GS_INFO("[CHASSIS] target_yaw = %lf,rest_yaw= %lf,sum_yaw = %lf", target_yaw,rest_yaw,sum_yaw);

       pre_yaw = current_yaw;
       if (rest_yaw < yaw_offset) {
           start_rotate_flag = false;
           is_rotate_finished = true;
           stop_rotate_flag = true;
           p_chassis_mcu->setTwoWheelSpeed(0.0, 0.0);
           std_msgs::Int32 msg;
           msg.data = 1;
           p_publisher->getRotateFinishedPub().publish(msg);

           timeval tv;
           gettimeofday(&tv, NULL);
           last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
       }else{
           double speed_w = p_speed_w->inplace_rotating_theta * rotate_angle/abs(rotate_angle);
           if (rest_yaw < 0.40) speed_w = fabs(rest_yaw) / 0.4 * speed_w;
           speed_w = (fabs(speed_w) < 0.07) ? speed_w/fabs(speed_w) * 0.07 : speed_w;

           is_rotate_finished = false;
           p_chassis_mcu->setTwoWheelSpeed(0.0, speed_w);
       }
       return true;
}

/*
 * 走直线测试
*/
void DoGoLine(void){

    double spe = p_speed_v->m_speed_v;
    current_pose = sqrt(fabs(g_odom_x*g_odom_x) + fabs(g_odom_y*g_odom_y));
    double rest_dis = distance - fabs(start_pose - current_pose);
    if (rest_dis < 0.03) {
        p_chassis_mcu->setTwoWheelSpeed(0.0,0.0);
        start_goline_flag = false;
        stop_goline_flag  = true;
        p_speed_v->m_speed_v = 0.0;
    }
    if (rest_dis < 0.15 && fabs(spe) < 0.3) {
        spe = rest_dis / 0.15 * spe;
    } else if(rest_dis < 0.25 && fabs(spe) > 0.3){
        spe = rest_dis / 0.5 * spe;
    }
    spe = (fabs(spe) < 0.04) ? 0.04*spe/fabs(spe) : spe;
    p_chassis_mcu->setTwoWheelSpeed(spe,0.0);
}

/*
 * 自动充电执行
*/
//void onCharge(void)
//{
//    if(sleep_cnt == 0){
//        charger_cmd_ = CMD_CHARGER_ON;
//        std::thread([&](){
//          GS_INFO("[CHASSIS] start to check charger volatage = %d", charger_voltage_);
//          sleep(3);
//          int check_charger_cnt = 0;
//          while(++sleep_cnt  < 60 && charger_cmd_ == CMD_CHARGER_ON) {
//           GS_INFO("[CHASSIS] checking charger volatage = %d", charger_voltage_);
//           if (charger_voltage_ >= p_charger->low_voltage) {
//             ++check_charger_cnt;
//           } else {
//             check_charger_cnt = 0;
//           }
//           if (check_charger_cnt > p_charger->delay_time && charger_cmd_ == CMD_CHARGER_ON) {
//             GS_INFO("[CHASSIS] check charger voltage normal > 30s, set charger relay on!!!");
//             p_chassis_mcu->setChargeCmd(CMD_CHARGER_ON);
//             pre_mileage = (p_chassis_mcu->mileage_right_ + p_chassis_mcu->mileage_left_) / 2;
//             break;
//           }
//            sleep(1);
//          }
//          sleep_cnt = 0; //允许下次线程能够进入
//        }).detach();
//    }
//}

/*
 * IO口控制，手触等
 */
void DoDIO(void) {
  unsigned int temp_di_data = p_chassis_mcu->doDIO(g_do_data_);
  GS_INFO("[wc_chassis] get_di data: 0x%x", temp_di_data);
  cur_emergency_status = (temp_di_data >> (8 + Emergency_stop)) & 0x01;
  // new version hardware hand touch
  if (p_hand_toucher->new_hand_touch) {
    temp_di_data = g_ultrasonic[0];
  }
  if (++g_dio_count > 2 && ((g_di_data_ & 0x03) != 0x03) && ((temp_di_data & 0x03) == 0x03)) {
    GS_INFO("[wc_chassis] get_di data: 0x%x, and then publish going back!!!", temp_di_data);
    std_msgs::UInt32 msg;
    msg.data = 1;
    p_publisher->getGoingBackPub().publish(msg);
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
