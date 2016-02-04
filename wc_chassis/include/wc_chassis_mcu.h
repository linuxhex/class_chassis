/* Copyright(C) Gaussian Robot. All rights reserved.
 */

#ifndef SRC_WC_CHASSIS_WC_CHASSIS_MCU_H_
#define SRC_WC_CHASSIS_WC_CHASSIS_MCU_H_

#include <ros/ros.h>
#include <string>
#include <map>

#include "MyDefine.h"
#include "TimerDiff.h"

//#define SPEED_THRESHOLD		4095
//#define PC_TEST_ONLY

class Socket;

class WC_chassis_mcu{
 public:
  WC_chassis_mcu();
  ~WC_chassis_mcu();

  void Init(const std::string& host_name, const std::string& port, float H, float Dia_F, float Dia_B, float Axle, float TimeWidth, int Counts, int Reduction_ratio, double Speed_ratio);

  void setThaZero(double zero);
  void setThaLeft(double left);

  void setDO(U32 usdo);
  unsigned int getDI();
  void setSpeed(float speed_v, float speed_w, int plan_type);
  void setTwoWheelSpeed(float speed_v, float speed_w);
  short getMotorSpeed(float speed);
  bool getOdo(double &x, double &y, double &a);  // NOLINT
  bool getCSpeed(double &v, double &w);  // NOLINT
  void getUltra();
  int getYawAngle();

  void comunication(void);

  bool setAuto(bool is_auto);
  bool is_Auto();
  int V2RPM(float v);
  int GetCopleySpeed(float v);
  int GetCopleyAngle(float angle);
  double acc_odom_theta_;
 private:
  bool is_auto_;

  int getLPos();
  int getRPos();

  float tha_zero_;

  float H_;
  float Dia_F_;
  float Dia_B_;
  float Axle_;
  float TimeWidth_;
  int Counts_;
  int Reduction_ratio_;
  double Speed_ratio_; 

  Socket* transfer_;

  bool first_odo_;
  double odom_x_;
  double odom_y_;
  double odom_a_;
  double odom_a_gyro_;
  double odom_v;
  double odom_w;

  S32 delta_counts_left_;
  S32 delta_counts_right_;
  S32 last_delta_counts_left_;
  S32 last_delta_counts_right_;
  S32 counts_left_;
  S32 counts_right_;
  S32 last_counts_left_;
  S32 last_counts_right_;
  short last_yaw_angle_;
  short yaw_angle_;
  float last_speed_v_;
  float last_speed_w_;

  ros::Publisher* pos_pub_left_;
  ros::Publisher* pos_pub_right_;

  U8 send_[10];
  U8 rec_[20];

  int direction;

  int last_angle_;
  int current_angle_;
  cTimerDiff dt;

  float speed_v_;
  float speed_w_;
};

#endif  // SRC_WC_CHASSIS_WC_CHASSIS_MCU_H_
