#ifndef __PARAMETER__
#define __PARAMETER__
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt32.h>
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
#include <autoscrubber_services/CheckProtectorStatus.h>
#include <autoscrubber_services/CheckChargeStatus.h>
#include <autoscrubber_services/SetChargeCmd.h>
#include <gs/logging.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <autoscrubber_services/TestGoLine.h>
#include <autoscrubber_services/GoLine.h>
#include <autoscrubber_services/StopGoLine.h>
#include <autoscrubber_services/CheckGoLine.h>
#include <pthread.h>
#include <thread>
#include <sched.h>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include <sstream>
#include <vector>
#include "param/speed_v.h"
#include "publish.h"
#include "subscribe.h"
#include "wc_chassis_mcu.h"
#include "param/machine.h"
#include "service.h"
#include "param/network.h"
#include "param/speed_w.h"
#include "param/charger.h"
#include "param/protector.h"
#include "param/hand_touch.h"
#include "param/ultrasonic.h"
#include "param/battery.h"
#include "param/checker_id.h"

extern  double g_odom_x   ;
extern  double g_odom_y   ;
extern  double g_odom_tha ;
extern  double g_odom_v ;
extern  double g_odom_w ;

enum Device_ID{
  Emergency_stop = 0,
  Mode,
  Battery,
  Mileage,
  Device_MAX
};
// Hardware ID
enum Hardware_ID {
  Main_board = 0,
  Pannel_board,
  Steering_board,
  Power_board,
  Ultrasonic_board,
  Motor_driver,
  Gyro_board,
  Motor,
  Ultrasonic,
  Hardware_MAX
};

typedef enum Protector_Hit_ {
  NONE_HIT = 0x00,
  FRONT_HIT = 0x01,
  REAR_HIT = 0x02,
  BOTH_HIT = 0x03,
} Protector_Hit;

typedef enum Relay_Cmd_ {
  CMD_CHARGER_STATUS    = 0x00,
  CMD_CHARGER_ON      = 0x01,
  CMD_CHARGER_OFF     = 0x02,
  CMD_CHARGER_MONITOR = 0x03,
  CMD_CTRL_PWR_ON     = 0x01 << 2,
  CMD_CTRL_PWR_OFF    = 0x02 << 2,
  CMD_MOTOR_PWR_ON    = 0x01 << 4,
  CMD_MOTOR_PWR_OFF   = 0x02 << 4,
  CMD_CUSTERM_PWR_ON  = 0x01 << 6,
  CMD_CUSTERM_PWR_OFF = 0x02 << 6,
} Relay_Cmd;

typedef enum Relay_Status_ {
  STA_CHARGER_ON      = 0x01,
  STA_CHARGER_OFF     = 0x02,
  STA_CHARGER_TOUCHED = 0x03,
  STA_CTRL_PWR_ON     = 0x01 << 2,
  STA_CTRL_PWR_OFF    = 0x02 << 2,
  STA_MOTOR_PWR_ON    = 0x01 << 4,
  STA_MOTOR_PWR_OFF   = 0x02 << 4,
  STA_CUSTERM_PWR_ON  = 0x01 << 6,
  STA_CUSTERM_PWR_OFF = 0x02 << 6,
} Relay_Status;

extern unsigned int protector_hit;
extern unsigned int g_dio_count ;
extern unsigned int g_ret_count ;
extern unsigned int g_pc_control;
extern double last_cmd_vel_time ;
extern int current_v_index;
extern int current_w_index;

extern unsigned int loop_count;
extern  int rotate_angle;
extern bool start_rotate_flag;
extern bool stop_rotate_flag;
extern bool is_rotate_finished;
extern bool old_ultrasonic_;
extern unsigned int g_di_data_;
extern unsigned int g_do_data_;
extern unsigned int cur_emergency_status;
extern unsigned int remote_ret_;
extern unsigned int charger_cmd_;
extern double charger_full_voltage_;
extern unsigned int remote_ret_cnt_;
extern unsigned char remote_cmd_;
extern unsigned short remote_index_;
extern std::string ultrasonic_str[15];
extern unsigned char special_ultrasonic_id[15];

extern std::vector<int> g_ultrasonic;
extern pthread_mutex_t speed_mutex;
extern int battery_count;
extern int charge_count;
extern int display_battery_capacity ;
extern int sum_battery_capacity;

extern bool socket_connection_status;
extern int ultrasonic_num;
extern double max_speed_w;
extern double speed_w_acc;
extern double speed_w_dec;
extern int    delta_counts_th;
extern int battery_level_;
extern double charger_voltage_;
extern double current_charge_value_;
extern int remote_id ;
extern std::string hardware_id;
extern bool laser_connection_status;
extern bool router_connection_status;
extern bool internet_connection_status;

extern std::thread *p_checkConnectionThread;
extern unsigned int protector_value;
extern unsigned int protector_bits;
extern unsigned int ultrasonic_bits;
extern bool ultrasonic_board_connection;
extern bool on_charge; //在充电true 停止充电false

extern unsigned int charger_status_;
extern unsigned int relay_status_;
extern double battery_value_;
extern double sum_battery_value_;
extern double charge_on_time_;
extern double go_forward_start_time_;
extern double protector_hit_time;  //防撞条触发开始时间
extern int charger_delay_time_;
extern std::string internet_url;
extern unsigned int sleep_cnt;

extern bool charger_relay; //充电继电器状态　false:打开，　true:闭合
extern bool inner_relay;  //内部继电器状态
extern bool outer_relay;  //外部继电器状态
extern bool user_relay;  //用户继电器状态

extern double pre_mileage;  //
extern double start_pose;
extern double current_pose;
extern double distance ;
extern bool start_goline_flag;
extern bool stop_goline_flag ;
extern double pre_yaw;
extern double sum_yaw;


extern Speed_v   *p_speed_v;
extern Speed_w   *p_speed_w;
extern Publisher *p_publisher;
extern Subscribe *p_subscribe;
extern Service   *p_service;
extern WC_chassis_mcu *p_chassis_mcu;
extern Machine     *p_machine;
extern Network     *p_network;
extern Charger     *p_charger;
extern Protector   *p_protector;
extern HandToucher *p_hand_toucher;
extern Ultrasonicer    *p_ultrasonic;
extern Param::Battery  *p_battery;
extern Checker_id      *p_checker_id;



#endif
