/* Copyright(C) Gaussian Robot. All rights reserved.
 */

#ifndef SRC_WC_CHASSIS_WC_CHASSIS_MCU_H_
#define SRC_WC_CHASSIS_WC_CHASSIS_MCU_H_

#include <ros/ros.h>
#include <string>
#include <map>
#include <gslib/gaussian_debug.h>

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

  unsigned int doDIO(unsigned int usdo);
  unsigned int checkRemoteVerifyKey(unsigned int seed_key);
  void setRemoteID(unsigned char id);
  void setRemoteRet(unsigned short ret);
  unsigned int getDI();
  void setSpeed(float speed_v, float speed_w, int plan_type);
  void setTwoWheelSpeed(float speed_v, float speed_w);
  short getMotorSpeed(float speed);
  bool getOdo(double &x, double &y, double &a);  // NOLINT
  bool getCSpeed(double &v, double &w);  // NOLINT
  void getRemoteCmd(unsigned char& cmd, unsigned short& index);
//  bool setRemoteStatus(unsigned int cmd, unsigned int mark);  // NOLINT
  void getUltra(void);
//  int getYawAngle(void);
  void getYawAngle(short& yaw, short& pitch, short& roll);
  void comunication(void);

  int V2RPM(float v);
  int GetCopleySpeed(float v);
  int GetCopleyAngle(float angle);
  short yaw_angle_;
  short pitch_angle_;
  short roll_angle_;
  double acc_odom_theta_;
  unsigned int gyro_state_;
  double mileage_left_;
  double mileage_right_;

 private:
  int getLPos();
  int getRPos();

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
