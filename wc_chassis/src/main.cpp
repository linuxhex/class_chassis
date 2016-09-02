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

    if(!old_ultrasonic_){  //旧板子没有这些功能
        updateDeviceStatus();
    }
    gettimeofday(&tv, NULL);
    double time_now = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

    if(start_rotate_flag) {
        if (charger_voltage_ > charger_low_voltage_) {
             go_forward_start_time_ = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
             charger_cmd_ = CMD_CHARGER_OFF;
             p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
         } else {
             go_forward_start_time_ = time_now + 1.0;
         }
        if (time_now - go_forward_start_time_ > -0.0001 && time_now - go_forward_start_time_ < 1.0 && !(protector_hit & FRONT_HIT)) {
            p_chassis_mcu->setTwoWheelSpeed(0.15, 0.0);
         } else {
            DoRotate(p_publisher->getRotateFinishedPub());
         }
    } else {
      if ((!laser_connection_status || !socket_connection_status)
          || (time_now - last_cmd_vel_time >= max_cmd_interval)
          || ((protector_hit & FRONT_HIT) && m_speed_v > 0.001)
          || ((protector_hit & REAR_HIT)  && m_speed_v < -0.001)
          || (charger_status_ == STA_CHARGER_ON)
          || (charger_status_ == STA_CHARGER_TOUCHED && m_speed_v < -0.001)) {

          if (charger_status_ == STA_CHARGER_ON && m_speed_v > 0.001) {
                 charger_cmd_ = CMD_CHARGER_OFF;
                 p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
          }
          p_chassis_mcu->setTwoWheelSpeed(0.0,0.0);

      } else {
          p_chassis_mcu->setTwoWheelSpeed(m_speed_v, m_speed_w);
      }
    }

    DoDIO(p_publisher->getGoingBackPub());
    DoRemoteRet();
    if (++loop_count % 2) {
      p_chassis_mcu->getRemoteCmd(remote_cmd_, remote_index_);
      p_publisher->publishRemoteCmd(remote_cmd_, remote_index_);
      p_publisher->publishDeviceStatus();
    }
    if (loop_count % 10) {//设置遥控器id
      p_chassis_mcu->setRemoteID((unsigned char)((remote_id & 0x0f) | ((remote_speed_level_ & 0x03) << 4) | ((battery_level_ & 0x03) << 6)));
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

