/* Copyright(C) Gaussian Robot. All rights reserved.
*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include "wc_chassis_mcu.h"
#include "init.h"
#include "parameter.h"
#include "publish.h"
#include "action.h"
#include "parameter.h"
#include "common_function.h"

ros::Publisher ultrasonic_pub[15];
ros::Publisher yaw_pub;
ros::Publisher odom_pub;
ros::Publisher gyro_pub;
ros::Publisher remote_cmd_pub;
ros::Publisher device_pub;
ros::Publisher rotate_finished_pub;
ros::Publisher protector_pub;
ros::Publisher going_back_pub;
ros::Publisher dio_pub;



int main(int argc, char **argv) {

    SetSchedPriority();   //设置chassis进程运行的优先级和占用的cpu核
    ROS_INFO("[wc_chassis] chassis version: 1.1.3.2");
    InitChassis(argc, argv,"wc_chassis");
    GAUSSIAN_INFO("[wc_chassis] chassis init completed");

    /*********************************publish handle init ******************************/
    yaw_pub         = p_n->advertise<std_msgs::Float32>("yaw", 10);
    odom_pub        = p_n->advertise<nav_msgs::Odometry>("odom", 50);
    gyro_pub        = p_device_nh->advertise<sensor_msgs::Imu>("gyro", 50);
    remote_cmd_pub  = p_device_nh->advertise<std_msgs::UInt32>("remote_cmd", 50);
    device_pub      = p_device_nh->advertise<diagnostic_msgs::DiagnosticStatus>("device_status", 50);
    rotate_finished_pub = p_device_nh->advertise<std_msgs::Int32>("rotate_finished", 5);
    protector_pub   = p_device_nh->advertise<diagnostic_msgs::KeyValue>("protector", 50);
    going_back_pub  = p_device_nh->advertise<std_msgs::UInt32>("cmd_going_back", 50);
    dio_pub         = p_device_nh->advertise<std_msgs::UInt32>("dio_data", 50);

    for(int i=0;i<15;i++){
      if(ultrasonic->find(ultrasonic_str[i]) != std::string::npos){
        ultrasonic_pub[i] = p_n->advertise<sensor_msgs::Range>(ultrasonic_str[i].c_str(), 50);
        if(special_ultrasonic->find(ultrasonic_str[i]) != std::string::npos){
          special_ultrasonic_id[i] = i;
        }
        ultrasonic_num ++;
      }
    }

   timeval tv;
    /*********************************  主循环  ******************************/
   while (ros::ok()) {
    g_chassis_mcu->getOdo(g_odom_x, g_odom_y, g_odom_tha);
    g_chassis_mcu->getCSpeed(g_odom_v, g_odom_w);
    g_chassis_mcu->getUltra();

    gettimeofday(&tv, NULL);
    double time_now = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

    if(start_rotate_flag) {
      DoRotate(rotate_finished_pub);
    } else {
      if ((time_now - last_cmd_vel_time >= max_cmd_interval) ||
          ((protector_hit & FRONT_HIT) && m_speed_v > 0.001) || 
          ((protector_hit & REAR_HIT)  && m_speed_v < -0.001) || 
          (current_charge_value_ > charger_low_voltage_)) {
        if (current_charge_value_ > charger_low_voltage_) {
          GAUSSIAN_INFO("WC CHASSIS: charge_voltage = %lf > charger_low_voltage = %lf", current_charge_value_, charger_low_voltage_); 
        }
        g_chassis_mcu->setTwoWheelSpeed(0.0,0.0);
      } else {
        protector_down = 0;
        g_chassis_mcu->setTwoWheelSpeed(m_speed_v, m_speed_w);
      }
    }

    DoDIO(going_back_pub);
    DoRemoteRet();
    if (++loop_count % 2) {
      g_chassis_mcu->getRemoteCmd(remote_cmd_, remote_index_);
      PublishRemoteCmd(remote_cmd_pub,remote_cmd_, remote_index_);
      publishDeviceStatus(device_pub);
    }
    if (loop_count % 10) {//设置遥控器id
      g_chassis_mcu->setRemoteID((unsigned char)((remote_id & 0x0f) | ((remote_speed_level_ & 0x03) << 4) | ((battery_level_ & 0x03) << 6)));
      loop_count = 0;
    }
    PublishDIO(dio_pub);
    PublishOdom(p_odom_broadcaster,odom_pub);
    PublishYaw(yaw_pub);
    PublishGyro(gyro_pub);
    PublishUltrasonic(ultrasonic_pub);
    publish_protector_status(protector_pub);
    ros::spinOnce();
    p_loop_rate->sleep();
  }

  GAUSSIAN_INFO("[wc_chassis] wc_chassis has closed, now free resource!");
  freeResource();
  return 0;
}

