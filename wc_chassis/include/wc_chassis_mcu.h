/* Copyright(C) Gaussian Robot. All rights reserved.
 */

#ifndef SRC_WC_CHASSIS_WC_CHASSIS_MCU_H_
#define SRC_WC_CHASSIS_WC_CHASSIS_MCU_H_

#include <ros/ros.h>
#include <string>
#include <map>

#include "TimerDiff.h"
#include "parameter.h"

class Socket;

class WC_chassis_mcu{
 public:
   WC_chassis_mcu();
  ~WC_chassis_mcu();

  void Init(const std::string& host_name, const std::string& port, float H, float Dia_F, float Dia_B, float Axle, float TimeWidth, int Counts, int Reduction_ratio, double Speed_ratio, double max_speed_v, double max_speed_w, double speed_v_acc, double speed_v_dec, double speed_v_dec_zero, double speed_w_acc, double speed_w_dec,double full_speed,int delta_counts_th);

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

  float  H_;
  float  Dia_F_;
  float  Dia_B_;
  float  Axle_;
  float  TimeWidth_;
  int    Counts_;
  int    Reduction_ratio_;
  double Speed_ratio_; 
  double max_speed_v_, max_speed_w_;
  double speed_v_acc_, speed_v_dec_, speed_v_dec_zero_;
  double speed_w_acc_, speed_w_dec_;
  double full_speed_;
  int delta_counts_th_;

  Socket* transfer_;

  bool   first_odo_;
  double odom_x_;
  double odom_y_;
  double odom_a_;
  double odom_a_gyro_;
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

  ros::Publisher* pos_pub_left_;
  ros::Publisher* pos_pub_right_;

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
