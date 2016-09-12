/* Copyright(C) Gaussian Robot. All rights reserved.
*/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "init.h"
#include "publish.h"
#include "action.h"
#include "parameter.h"
#include "common_function.h"
#include "data_process.h"

int main(int argc, char **argv) {

    SetSchedPriority();   //设置chassis进程运行的优先级和占用的cpu核

    ROS_INFO("[wc_chassis] chassis version: 1.1.4.6");
    InitChassis(argc, argv,"wc_chassis");
    GS_INFO("[wc_chassis] chassis init completed");

    timeval tv;

    /*********************************  主循环  ******************************/
   while (ros::ok()) {

    p_chassis_mcu->getOdo(g_odom_x, g_odom_y, g_odom_tha);
    p_chassis_mcu->getCSpeed(g_odom_v, g_odom_w);
    p_chassis_mcu->getUltra();

    updateDeviceStatus();
    gettimeofday(&tv, NULL);
    double time_now = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

    if(start_goline_flag){
        DoGoLine();
    }else if(start_rotate_flag) {
        if(p_charger != NULL){
            if (p_charger->charger_voltage > p_charger->low_voltage) {
                 p_charger->go_forward_start_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
                 p_charger->charger_cmd = CMD_CHARGER_OFF;
                 p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
             } else {
                 p_charger->go_forward_start_time = time_now + 1.0;
             }
        }
        if ((p_charger !=NULL) && (time_now - p_charger->go_forward_start_time > -0.0001) && time_now - p_charger->go_forward_start_time < 1.0 && !(p_protector->protector_hit & FRONT_HIT)) {
            p_chassis_mcu->setTwoWheelSpeed(0.15, 0.0);
         } else {
            DoRotate();
         }
    } else {
      if ((!p_network->laser_connection_status || !p_network->socket_connection_status)
          || (time_now - last_cmd_vel_time >= p_machine->max_cmd_interval)
          || ((p_protector !=NULL) && (p_protector->protector_hit & FRONT_HIT) && p_speed_v->m_speed_v > 0.001)
          || ((p_protector !=NULL) && (p_protector->protector_hit & REAR_HIT)  && p_speed_v->m_speed_v < -0.001)
          || ((p_charger != NULL) && p_charger->charger_status == STA_CHARGER_ON)
          || ((p_charger != NULL) && p_charger->charger_status == STA_CHARGER_TOUCHED && p_speed_v->m_speed_v < -0.001)) {

          if ((p_charger != NULL) && (p_charger->charger_status == STA_CHARGER_ON) && p_speed_v->m_speed_v > 0.001) {
                 p_charger->charger_cmd = CMD_CHARGER_OFF;
                 p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
          }
          p_chassis_mcu->setTwoWheelSpeed(0.0,0.0);

      } else {
          p_chassis_mcu->setTwoWheelSpeed(p_speed_v->m_speed_v, p_speed_w->m_speed_w);
      }
    }

    DoDIO();
    DoRemoteRet();
    if (++loop_count % 2) {
      p_chassis_mcu->getRemoteCmd(remote_cmd_, remote_index_);
      p_publisher->publishRemoteCmd(remote_cmd_, remote_index_);
      p_publisher->publishDeviceStatus();
    }
    if (loop_count % 10) {//设置遥控器id
      p_chassis_mcu->setRemoteID((unsigned char)((remote_id & 0x0f) | ((p_speed_v->remote_level & 0x03) << 4) | ((p_battery->battery_level & 0x03) << 6)));
      loop_count = 0;
    }
    p_publisher->publishDIO();
    p_publisher->publishOdom(p_odom_broadcaster);
    p_publisher->publishYaw();
    p_publisher->publishUltrasonics();
    p_publisher->publishProtectorStatus();
    p_publisher->publishProtectorValue();
    ros::spinOnce();
    p_loop_rate->sleep();
  }

  GS_INFO("[wc_chassis] wc_chassis has closed, now free resource!");
  freeResource();
  return 0;
}

