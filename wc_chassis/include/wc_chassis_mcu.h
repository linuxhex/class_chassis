/* Copyright(C) Gaussian Robot. All rights reserved.
 */

#ifndef SRC_WC_CHASSIS_WC_CHASSIS_MCU_H_
#define SRC_WC_CHASSIS_WC_CHASSIS_MCU_H_

#include <ros/ros.h>
#include <string>
#include <map>

#include "TimerDiff.h"

class Socket;

class WC_chassis_mcu{
 public:
   WC_chassis_mcu();
  ~WC_chassis_mcu();

  void Init(const std::string& host_name, const std::string& port,  double Speed_ratio, double max_speed_v, double max_speed_w, double speed_v_acc, double speed_v_dec, double speed_v_dec_zero, double speed_w_acc, double speed_w_dec,double full_speed);

  void setThaZero(double zero);
  void setThaLeft(double left);

  unsigned int doDIO(unsigned int usdo);
  void setRemoteRet(unsigned short ret);
  unsigned int getDI();
  void  setTwoWheelSpeed(float speed_v, float speed_w);
  short getMotorSpeed(float speed);
  bool  getOdo(double &x, double &y, double &a);  // NOLINT
  bool  getCSpeed(double &v, double &w);  // NOLINT
  void  getRemoteCmd(unsigned char& cmd, unsigned short& index);
  void getUltra(void);
  void setRemoteID(unsigned char id);
  void getYawAngle(short& yaw, short& pitch, short& roll);
  void comunication(void);
  unsigned int checkRemoteVerifyKey(unsigned int seed_key);
  unsigned char setChargeCmd(unsigned char cmd);
  void setShutdownCmd(unsigned char cmd);
  void yawSwitch(void);
  unsigned int getCntTime(void);
  int V2RPM(float v);
  int GetCopleySpeed(float v);
  int GetCopleyAngle(float angle);
  short yaw_angle_;
  short pre_yaw_angle_;
  short sum_delta_yaw_angle_;
  unsigned int yaw_count_;
  short pitch_angle_;
  short roll_angle_;
  double acc_odom_theta_;
  unsigned int gyro_state_;
  double mileage_left_;
  double mileage_right_;

 private:
  int getLPos();
  int getRPos();
  int    Counts_;
  double Speed_ratio_; 
  double max_speed_v_, max_speed_w_;
  double speed_v_acc_, speed_v_dec_, speed_v_dec_zero_;
  double speed_w_acc_, speed_w_dec_;
  double full_speed_;

  Socket* transfer_ = NULL;

  bool   first_odo_;
  double odom_x_;
  double odom_y_;
  double odom_a_;
  double odom_v;
  double odom_w;

  int delta_counts_left_;
  int delta_counts_right_;
  int last_delta_counts_left_;
  int last_delta_counts_right_;
  int last_odo_delta_counts_left_;
  int last_odo_delta_counts_right_;
  int counts_left_;
  int counts_right_;
  int last_counts_left_;
  int last_counts_right_;
  short last_yaw_angle_;
  float last_speed_v_;
  float last_speed_w_;

  unsigned char send_[10];
  unsigned char rec_[20];

  int direction;

  int last_angle_;
  int current_angle_;
  cTimerDiff dt;

  float speed_v_;
  float speed_w_;
};

#endif  // SRC_WC_CHASSIS_WC_CHASSIS_MCU_H_
