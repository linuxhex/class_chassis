/* Copyright(C) Gaussian Robot. All rights reserved.
 */

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <pthread.h>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include "SPort.h"
#include "protocol.h"  // NOLINT

#include "wc_chassis_mcu.h"  // NOLINT

#define SPEED_CMD_TH	          (1000)	
#define SPEED_V_TH		(0.52)
#define SPEED_W_TH		(0.6)
#define DELTA_SPEED_V_INC_TH    (0.025)
#define DELTA_SPEED_V_DEC_TH    (-0.12)
#define DELTA_SPEED_W_TH	(0.25) 
#define DT                      (0.1)

const float  H = 0.92;
float current_v = 0.0;
float current_theta = 0.0;
double GetTimeInSeconds() {
  timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec + 0.000001 * t.tv_usec;
}

WC_chassis_mcu::WC_chassis_mcu()
  : transfer_(0), H_(0.5), Dia_F_(0.2), Dia_B_(0.2), Axle_(0.6), Counts_(4000), Reduction_ratio_(25), Speed_ratio_(1.0),
    odom_x_(0.0), odom_y_(0.0), odom_a_(0.0), odom_a_gyro_(0.0), acc_odom_theta_(0.0),
    delta_counts_left_(0), delta_counts_right_(0), mileage_left_(0.0), mileage_right_(0.0),
    yaw_angle_(0), pitch_angle_(0), roll_angle_(0),
    last_speed_v_(0.0), last_speed_w_(0.0),
    counts_left_(0), counts_right_(0), first_odo_(true),
    direction(0), speed_v_(0), speed_w_(0), gyro_state_(0) {
  memset(send_, 0, 10);
  memset(rec_, 0, 20);
}

WC_chassis_mcu::~WC_chassis_mcu() { }

void WC_chassis_mcu::Init(const std::string& host_name, const std::string& port, float H, float Dia_F, float Dia_B, float Axle, float TimeWidth, int Counts, int Reduction_ratio, double Speed_ratio, double max_speed_v, double max_speed_w, double speed_v_acc, double speed_v_dec, double speed_v_dec_zero, double speed_w_acc, double speed_w_dec,double full_speed,int delta_counts_th) {
  if (!transfer_) {
    transfer_ = new Socket();
    transfer_->Init(host_name, port);
  }

  if ((H > 0) && (H < 2.0)) {
    H_ = H;
  } else {
    std::cout << "H err value:" <<H<< std::endl;
  }
  if ((Dia_F > 0) && (Dia_F < 0.5)) {
    Dia_F_ = Dia_F;
  } else {
    std::cout << "Dia_F err value:" <<Dia_F<< std::endl;
  }

  if ((Dia_B > 0) && (Dia_B < 0.5)) {
    Dia_B_ = Dia_B;
  } else {
    std::cout << "Dia_B err value:" <<Dia_B<< std::endl;
  }

  if ((Axle > 0) && (Axle < 1.0)) {
    Axle_ = Axle;
  } else {
    std::cout << "Axle err value:" <<Axle<< std::endl;
  }

  if ((Counts > 0) && (Counts < 11000)) {
    Counts_ = Counts;
  } else {
    std::cout << "Counts err value:" <<Counts<< std::endl;
  }

  if ((Reduction_ratio > 0) && (Reduction_ratio < 100)) {
    Reduction_ratio_ = Reduction_ratio;
  } else {
    std::cout << "Reduction_ratio err value:" << Reduction_ratio<< std::endl;
  }

  if ((Speed_ratio > 0) && (Speed_ratio < 2)) {
    Speed_ratio_ = Speed_ratio;
  } else {
    std::cout << "Speed_ratio err value:" << Speed_ratio<< std::endl;
  }

  if ((TimeWidth > 0) && (TimeWidth < 0.2)) {
    TimeWidth_ = TimeWidth;
  } else {
    std::cout << "TimeWidth err value:" <<TimeWidth<< std::endl;
  }

  if (max_speed_v > 0) {
    max_speed_v_ = max_speed_v;
  } else {
    max_speed_v_ = 0.5;
    std::cout << "max_speed_v err value:" <<max_speed_v<< std::endl;
  }

  if (max_speed_w > 0) {
    max_speed_w_ = max_speed_w;
  } else {
    max_speed_w_ = 0.5;
    std::cout << "max_speed_w err value:" <<max_speed_w<< std::endl;
  }

  if ((speed_v_acc > 0) && (speed_v_acc < max_speed_v_)) {
    speed_v_acc_ = speed_v_acc;
  } else {
    speed_v_acc_ = 0.05;
    std::cout << "speed_v_acc err value:" <<speed_v_acc<< std::endl;
  }

  if ((speed_v_dec < 0) && (speed_v_dec > (-1.0 * max_speed_v_))) {
    speed_v_dec_ = speed_v_dec;
  } else {
    speed_v_dec_ = -0.12;
    std::cout << "speed_v_dec err value:" <<speed_v_dec<< std::endl;
  }

  if ((speed_v_dec_zero < 0) && (speed_v_dec_zero > (-1.0 * max_speed_v_))) {
    speed_v_dec_zero_ = speed_v_dec_zero;
  } else {
    speed_v_dec_zero_ = -0.12;
    std::cout << "speed_v_dec_zero err value:" <<speed_v_dec_zero<< std::endl;
  }

  if ((speed_w_acc > 0) && (speed_w_acc < max_speed_w_)) {
    speed_w_acc_ = speed_w_acc;
  } else {
    speed_w_acc_ = 0.25;
    std::cout << "speed_w_acc err value:" <<speed_w_acc<< std::endl;
  }

  if ((speed_w_dec < 0) && (speed_w_dec > (-1.0 * max_speed_w_))) {
    speed_w_dec_ = speed_w_dec;
  } else {
    speed_w_dec_ = -0.25;
    std::cout << "speed_w_dec err value:" <<speed_w_dec<< std::endl;
  }

  if ((full_speed > 0) && (full_speed < 20)) {
    full_speed_ = full_speed;
  } else {
    full_speed_ = 3.0;
    std::cout << "full_speed err value:" <<full_speed<< std::endl;
  }
  if ((delta_counts_th > 0) && (delta_counts_th < 200)) {
    delta_counts_th_ = delta_counts_th;
  } else {
    delta_counts_th_ = 40;
    std::cout << "delta_counts_th err value:" <<delta_counts_th<< std::endl;
  }

}

int WC_chassis_mcu::V2RPM(float v) {
  int rpm = v * 60 / (M_PI*Dia_F_);
  return rpm;
}

bool WC_chassis_mcu::getCSpeed(double &v, double &w) {
  if (abs(delta_counts_left_) > 800) {
    std::cout << "err delta_counts_left_:" << delta_counts_left_ << std::endl;
    delta_counts_left_ = last_delta_counts_left_;
  } else {
    last_delta_counts_left_ = delta_counts_left_;
  }
  if (abs(delta_counts_right_) > 800) {
    std::cout << "err delta_counts_right_:" << delta_counts_right_ << std::endl;
    delta_counts_right_ = last_delta_counts_right_;
  } else {
    last_delta_counts_right_ = delta_counts_right_;
  }

  const double t = 0.05; //stm32 update delta_counts_ in 20Hz
  double l_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_left_ * M_PI) / (Counts_ * Reduction_ratio_);  // 200000;  // 81920
  double r_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_right_ * M_PI) / (Counts_ * Reduction_ratio_);  // 200000;  // 81920
  mileage_left_ += fabs(l_wheel_pos);
  mileage_right_ += fabs(r_wheel_pos);

  double dx = (r_wheel_pos + l_wheel_pos) * 0.5;
  double da = (r_wheel_pos - l_wheel_pos) / Axle_;

  if ((fabs(t) > 10e-3) && (fabs(t) < 10e3)) {
    v = dx / t * Speed_ratio_;
    w = da / t;
  } else {
    v = 0;
    w = 0;
  }
//  ROS_INFO("get v = %lf, w = %lf; set v = %lf, w = %lf",v, w, speed_v_, speed_w_);
  return true;
}

int getsign(int t) {
  return t >= 0 ? (1) : (-1);
}

bool WC_chassis_mcu::getOdo(double &x, double &y, double &a) {
  comunication();

  // std::cout << "left: " << counts_left_ << " right: " << counts_right_ << " dleft: " << delta_counts_left_ << " dright: " << delta_counts_right_ << " angle: " << yaw_angle_  << std::endl;
	ROS_INFO("[WC CHASSIS] left: %d right: %d dleft: %d dright: %d ddleft: %d ddright: %d ngle: %f", counts_left_, counts_right_, delta_counts_left_, delta_counts_right_, delta_counts_left_ - last_odo_delta_counts_left_, delta_counts_right_ - last_odo_delta_counts_right_, yaw_angle_ / 10.0);

  if (first_odo_) {
    odom_x_ = 0;
    odom_y_ = 0;
    odom_a_ = 0;
    odom_a_gyro_ = 0;
    acc_odom_theta_ = 0.0;

    last_odo_delta_counts_right_ = delta_counts_right_;
    last_odo_delta_counts_left_ = delta_counts_left_;

    last_counts_left_ = counts_left_;
    last_counts_right_ = counts_right_;
    last_yaw_angle_ = yaw_angle_;
    if ((abs(last_counts_left_) > 0) && (abs(last_counts_right_) > 0)) {
      first_odo_ = false;
    }
    return false;
  }
  int critical_delta = 2;
  double odom_a_ratio = 1.0; // 0.963;
  // double round_left = (double)counts_left_ / Counts_;
  // double round_right = (double)counts_right_ / Counts_;
//  ROS_INFO("left turn roudn = %.4f ", round_left);
//  ROS_INFO("right turn roudn = %.4f ", round_right);
  // std::cout << "left turn round: " << showpoint << round_left << std::endl; 
  // std::cout << "right turn round: " << showpoint << round_right << std::endl;
 
  int delta_counts_left = (counts_left_ - last_counts_left_) * getsign(delta_counts_left_);
  int delta_counts_right = (counts_right_ - last_counts_right_) * getsign(delta_counts_right_);

  if (delta_counts_left > 800) {
    delta_counts_left -= 65536;
  } else if (delta_counts_left < -2000) {
    delta_counts_left += 65536;
  }
  if (delta_counts_right > 800) {
    delta_counts_right -= 65536;
  } else if (delta_counts_right < -2000) {
    delta_counts_right += 65536;
  }

  // std::cout << "dleft: " << delta_counts_left << " dright: " << delta_counts_right << std::endl;

  //防止码盘抖动
  if (abs(delta_counts_right) > delta_counts_th_) {
    ROS_ERROR("err delta_counts_right: %d", delta_counts_right);
    delta_counts_right = last_odo_delta_counts_right_;
  }
  if (abs(delta_counts_left) > delta_counts_th_) {
    ROS_ERROR("err delta_counts_left: %d", delta_counts_left);
    delta_counts_left = last_odo_delta_counts_left_;
  }

	ROS_INFO("[WC CHASSIS] gotOdo:: delta_counts_left: %d delta_counts_right: %d last_odo_delta_counts_left_: %d last_odo_delta_counts_right_: %d", delta_counts_left, delta_counts_right, last_odo_delta_counts_left_, last_odo_delta_counts_right_);

  last_odo_delta_counts_right_ = delta_counts_right;
  last_odo_delta_counts_left_ = delta_counts_left;

  double l_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_left * M_PI) / (Counts_ * Reduction_ratio_);  // 200000;  // 81920
  double r_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_right * M_PI) / (Counts_ * Reduction_ratio_);  // 200000;  // 81920

  double dx = (r_wheel_pos + l_wheel_pos) * 0.5;
  double da = (r_wheel_pos - l_wheel_pos) / Axle_ * odom_a_ratio;
//  double da = asin((r_wheel_pos - l_wheel_pos) / Axle_);
  odom_x_ += dx * cos(odom_a_);
  odom_y_ += dx * sin(odom_a_);
//  odom_a_ += da;
//  acc_odom_theta_ += da;

//  if (!(abs(delta_counts_left) < critical_delta && abs(delta_counts_right) < critical_delta)) {
  if (!gyro_state_ || (gyro_state_ && !(abs(delta_counts_left) < critical_delta && abs(delta_counts_right) < critical_delta))) {
    double temp_dtheta = yaw_angle_ - last_yaw_angle_;
    if(temp_dtheta > 3000.0) {
      temp_dtheta = -1.0 * (3600.0 - temp_dtheta);
    } else if(temp_dtheta <= -3000.0) {
      temp_dtheta = 3600.0 + temp_dtheta;
    }

    double gyro_dtheta =  (temp_dtheta / 10.0) / 180.0 * M_PI;
    odom_a_ += gyro_dtheta;
    acc_odom_theta_ += fabs(gyro_dtheta);
//    std::cout << "temp_theta: " << temp_dtheta << " ;odom_dtheta: " << gyro_dtheta << " acc_odom_theta_: " << acc_odom_theta_ << std::endl;
    odom_a_gyro_ = odom_a_; 
  }

  while (odom_a_ <= - M_PI) {
    odom_a_ += M_PI*2;
  }
  while (odom_a_ > M_PI) {
    odom_a_ -= M_PI*2;
  }
  while (odom_a_gyro_ <= - M_PI) {
    odom_a_gyro_ += M_PI*2;
  }
  while (odom_a_gyro_ > M_PI) {
    odom_a_gyro_ -= M_PI*2;
  }

  double yaw = odom_a_ * 180.0 / M_PI;
  //ROS_INFO("yaw angle = %lf", yaw);
  //ROS_INFO("odom_a=%lf, odom_a_gyro=%lf", odom_a_ * 57.3, odom_a_gyro_ * 57.3); 
  x = odom_x_;
  y = odom_y_;
  a = odom_a_;

  last_counts_left_ = counts_left_;
  last_counts_right_ = counts_right_;
  last_yaw_angle_ = yaw_angle_;
  return true;
}

void WC_chassis_mcu::getUltra() {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateRUltra(send, &len);
  double start_time = GetTimeInSeconds();
  if (transfer_) {
    transfer_->Send_data(send, len);
#ifdef DEBUG_ETHERNET
    double send_time = GetTimeInSeconds(); 
    if (send_time - start_time> 0.002)
      ROS_INFO("[CHASSIS] get_Ultra: send time = %lf", send_time - start_time);
#endif

    transfer_->Read_data(rec, rlen, 23, 500);
#ifdef DEBUG_ETHERNET
    double recv_time = GetTimeInSeconds();
    if (recv_time - send_time> 0.002)
      ROS_INFO("[CHASSIS] get_Ultra: recv time = %lf", recv_time - send_time);
#endif
  }

  // std::string str = cComm::ByteToHexString(send, len);
  // std::cout << "send ultra: " << str << std::endl;

   std::string str = cComm::ByteToHexString(rec, rlen);
   std::cout << "recv ultra: " << str << std::endl;
  // std::cout << "recv right pos:  " << str.substr(30, 12) << std::endl;
  if (rlen == 35) {
    for (int i = 0; i < rlen; ++i) {
      if (IRQ_CH(rec[i])) {
        return ;
      }
    }
  } else {
    // sleep(1);
  }
}

unsigned int WC_chassis_mcu::checkRemoteVerifyKey(unsigned int seed_key) {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateRemoteVerifyKey(send, &len, 0, seed_key);

  double start_time = GetTimeInSeconds();
  if (transfer_) {
    transfer_->Send_data(send, len);
#ifdef DEBUG_ETHERNET
    double send_time = GetTimeInSeconds(); 
    if (send_time - start_time > 0.002)
      ROS_INFO("[CHASSIS] checkRemoteVerifyKey: send time = %lf", send_time - start_time);
#endif
    transfer_->Read_data(rec, rlen, 15, 500);
#ifdef DEBUG_ETHERNET
    double recv_time = GetTimeInSeconds();
    if (recv_time - send_time > 0.002)
      ROS_INFO("[CHASSIS] checkRemoteVerifyKey: recv time = %lf", recv_time - send_time);
#endif
  }

//  std::string str_send = cComm::ByteToHexString(send, len);
//  std::cout << "send Remote verify key: " << str_send << std::endl;

//  std::string str_recv = cComm::ByteToHexString(rec, rlen);
//  std::cout << "recv Remote verify key: " << str_recv << std::endl;
  if (rlen == 15) {
    for (int i = 0; i < rlen; ++i) {
      if (IRQ_CH(rec[i])) {
        return getRemoteVerifyKey();
        // ROS_INFO("[wc_chassis] cmd = %d; index = %d", cmd, index);
      }
    }
  } else {
    // sleep(1);
  }
  return 0;
}

void WC_chassis_mcu::getRemoteCmd(unsigned char& cmd, unsigned short& index) {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateRemoteCmd(send, &len);

  double start_time = GetTimeInSeconds();
  if (transfer_) {
    transfer_->Send_data(send, len);
#ifdef DEBUG_ETHERNET
    double send_time = GetTimeInSeconds(); 
    if (send_time - start_time> 0.002)
      ROS_INFO("[CHASSIS] get remote cmd: send time = %lf", send_time - start_time);
#endif
    transfer_->Read_data(rec, rlen, 14, 500);
#ifdef DEBUG_ETHERNET
    double recv_time = GetTimeInSeconds();
    if (recv_time - send_time> 0.002)
      ROS_INFO("[CHASSIS] get remote cmd: recv time = %lf", recv_time - send_time);
#endif
  }

  // std::string str = cComm::ByteToHexString(send, len);
  // std::cout << "send Remote cmd: " << str << std::endl;

  std::string str = cComm::ByteToHexString(rec, rlen);
  std::cout << "recv Remote cmd: " << str << std::endl;
  if (rlen == 14) {
    for (int i = 0; i < rlen; ++i) {
      if (IRQ_CH(rec[i])) {
        getRemote(cmd, index);
        // ROS_INFO("[wc_chassis] cmd = %d; index = %d", cmd, index);
      }
    }
  } else {
    // sleep(1);
  }
//  return 0;
}

void WC_chassis_mcu::getYawAngle(short& yaw, short& pitch, short& roll) {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateYawAngle(send, &len);

  double start_time = GetTimeInSeconds();
  if (transfer_) {
    transfer_->Send_data(send, len);
#ifdef DEBUG_ETHERNET
    double send_time = GetTimeInSeconds(); 
    if (send_time - start_time> 0.002)
      ROS_INFO("[CHASSIS] get Yaw Angle: send time = %lf", send_time - start_time);
#endif
    transfer_->Read_data(rec, rlen, 17, 500);
#ifdef DEBUG_ETHERNET
    double recv_time = GetTimeInSeconds();
    if (recv_time - send_time> 0.002)
      ROS_INFO("[CHASSIS] get Yaw Angle: recv time = %lf", recv_time - send_time);
#endif
  }

  // std::string str = cComm::ByteToHexString(send, len);
  // std::cout << "send yaw: " << str << std::endl;

  std::string str = cComm::ByteToHexString(rec, rlen);
  std::cout << "recv Yaw angle: " << str << std::endl;
  if (rlen == 17) {
    for (int i = 0; i < rlen; ++i) {
      if (IRQ_CH(rec[i])) {
        getYaw(yaw, pitch, roll);
        std::cout << "Yaw = " << yaw / 10.0 << "; Pitch = " << pitch / 10.0 << "; Roll = " << roll / 10.0 << std::endl;
      }
    }
  } else {
    // sleep(1);
  }
//	return 0;
}

int WC_chassis_mcu::getLPos() {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateRPos(send, &len, 0);

  double start_time = GetTimeInSeconds();
  if (transfer_) {
    transfer_->Send_data(send, len);
#ifdef DEBUG_ETHERNET
    double send_time = GetTimeInSeconds(); 
    if (send_time - start_time > 0.002)
      ROS_INFO("[CHASSIS] get left pos: send time = %lf", send_time - start_time);
#endif
    transfer_->Read_data(rec, rlen, 23, 500);
#ifdef DEBUG_ETHERNET
    double recv_time = GetTimeInSeconds();
    if (recv_time - send_time > 0.002)
      ROS_INFO("[CHASSIS] get left pos: recv time = %lf", recv_time - send_time);
#endif
  }

  // std::string str = cComm::ByteToHexString(send, len);
  // std::cout << "send left pos: " << str << std::endl;

   // std::string str = cComm::ByteToHexString(rec, rlen);
   // std::cout << "recv left pos:  " << str << std::endl;
  //   std::cout << "recv left pos:   " << str.substr(30, 12) << std::endl;

  if (rlen == 23) {
    for (int i = 0; i < rlen; ++i) {
      if (IRQ_CH(rec[i])) {
        counts_left_ = GetPos(0);
        return GetDelta(0);
      }
    }
  } else {
    // sleep(1);
  }

  return delta_counts_left_;
}

int WC_chassis_mcu::getRPos() {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateRPos(send, &len, 1);

  double start_time = GetTimeInSeconds();
  if (transfer_) {
    transfer_->Send_data(send, len);
#ifdef DEBUG_ETHERNET
    double send_time = GetTimeInSeconds(); 
    if (send_time - start_time> 0.002)
      ROS_INFO("[CHASSIS] get right pos: send time = %lf", send_time - start_time);
#endif

    transfer_->Read_data(rec, rlen, 23, 500);
#ifdef DEBUG_ETHERNET
    double recv_time = GetTimeInSeconds();
    if (recv_time - send_time> 0.002)
      ROS_INFO("[CHASSIS] get right pos: recv time = %lf", recv_time - send_time);
#endif
  }

  // std::string str = cComm::ByteToHexString(send, len);
  // std::cout << "send right pos: " << str << std::endl;

  // std::string str = cComm::ByteToHexString(rec, rlen);
  // std::cout << "recv right pos: " << str << std::endl;
  //  std::cout << "recv right pos:  " << str.substr(30, 12) << std::endl;

  if (rlen == 23) {
    for (int i = 0; i < rlen; ++i) {
      if (IRQ_CH(rec[i])) {
        counts_right_ = GetPos(1);
        return GetDelta(1);
      }
    }
  } else {
    // sleep(1);
  }
  return delta_counts_right_;
}

unsigned int WC_chassis_mcu::doDIO(unsigned int usdo) {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateDO(send, &len, 0, usdo);

  double start_time = GetTimeInSeconds();
  if (transfer_) {
    transfer_->Send_data(send, len);
#ifdef DEBUG_ETHERNET
    double send_time = GetTimeInSeconds(); 
    if (send_time - start_time> 0.002)
      ROS_INFO("[CHASSIS] set_DO: cost time = %lf", send_time - start_time);
#endif

    transfer_->Read_data(rec, rlen, 15, 500);
#ifdef DEBUG_ETHERNET
    double recv_time = GetTimeInSeconds();
    if (recv_time - send_time> 0.002)
      ROS_INFO("[CHASSIS] get_DI: cost time = %lf", recv_time - send_time);
#endif
  }
  std::string str_send = cComm::ByteToHexString(send, len);
  std::cout << "send do: " << str_send << std::endl;
  std::string str_recv = cComm::ByteToHexString(rec, len);
  std::cout << "get di: " << str_recv << std::endl;

  if (rlen == 15) {
    for (int i = 0 ; i < rlen ; ++i) {
      if (IRQ_CH(rec[i])) {
        return GetDI();
      }
    }
  } else {
    // sleep(1);
  }

  return 0xffffffff;
}

void WC_chassis_mcu::setRemoteID(unsigned char id) {
  unsigned char remote_id = id & 0x0f;  
  if (remote_id < 1 || remote_id > 9) { 
    ROS_ERROR("[wc_chassis] remote id = %d, beyond (1 ~ 9)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!", remote_id);
    return ;
  }
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;
  
  ROS_INFO("[wc_chassis] set chassis remote id = %d, speed_level = %d, power_level = %d", remote_id, (remote_id >> 4) & 0x03, (remote_id >> 6) & 0x03);


  CreateRemoteID(send, &len, 0, id);

  double start_time = GetTimeInSeconds();

  if (transfer_) {
    transfer_->Send_data(send, len);
#ifdef DEBUG_ETHERNET
    double send_time = GetTimeInSeconds(); 
    if (send_time - start_time> 0.002)
      ROS_INFO("[CHASSIS] setRemoteID: send time = %lf", send_time - start_time);
#endif
    transfer_->Read_data(rec, rlen, 12, 500);
#ifdef DEBUG_ETHERNET
    double recv_time = GetTimeInSeconds();
    if (recv_time - send_time> 0.002)
      ROS_INFO("[CHASSIS] setRemoteID: recv time = %lf", recv_time - send_time);
#endif
  }

  std::string str = cComm::ByteToHexString(send, len);
  std::cout << "send remote id: " << str << std::endl;

  usleep(1000);
}

void WC_chassis_mcu::setRemoteRet(unsigned short ret) {
  if (ret == 0) { 
    ROS_INFO("[wc_chassis] send remote ret == 0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;
  
  ROS_INFO("chassis remote ret cmd = %d, state = %d", (ret & 0xff), (ret >> 8) & 0xff);

  CreateRemoteRet(send, &len, 0, ret);

  double start_time = GetTimeInSeconds();

  if (transfer_) {
    transfer_->Send_data(send, len);
#ifdef DEBUG_ETHERNET
    double send_time = GetTimeInSeconds(); 
    if (send_time - start_time> 0.002)
      ROS_INFO("[CHASSIS] setRemoteRet: send time = %lf", send_time - start_time);
#endif
    transfer_->Read_data(rec, rlen, 12, 500);
#ifdef DEBUG_ETHERNET
    double recv_time = GetTimeInSeconds();
    if (recv_time - send_time> 0.002)
      ROS_INFO("[CHASSIS] setRemoteRet: recv time = %lf", recv_time - send_time);
#endif
  }

  std::string str = cComm::ByteToHexString(send, len);
  std::cout << "send remote ret: " << str << std::endl;

  usleep(1000);
}

bool IsInPlaceRotation(float v, float w) {
  if ((fabs(v) < 0.07 && fabs(w) > 0.25) ||
      (fabs(v) < 0.001 && fabs(w) >= 0.001)) {
    return true;
  } else {
    return false;
  }
}

bool IsStop(float v, float w) {
  if (fabs(v) < 0.001 && fabs(w) < 0.001) {
    return true;
  } else {
    return false;
  }
}

void CalculateAngleAndSpeed(float* angle, float* speed, float v, float w) {
  if (IsInPlaceRotation(v, w)) {	// v=Low & w=High -> Rotation Only
    if (w > 0) {
      *angle = M_PI_2;
    } else {
      *angle = -M_PI_2;
    }

    *speed = fabs(H * w);
  } else if (IsStop(v, w)) {
    *angle = 0.0;
    *speed = 0.0;
  } else {
    *angle = atan(H * w / v);
    *speed = v / cos(*angle);
  }
}

int WC_chassis_mcu::GetCopleySpeed(float v) {       // 将m/s转换成RPM
  // max speed
  v = v > 0.8 ? 0.8 : v;
  return v * 60 / (M_PI * Dia_F_);
}

int WC_chassis_mcu::GetCopleyAngle(float angle) {
  int ret = 1000 * (angle / M_PI_2);
  ret = ret > 1000 ? 1000 : ret;
  ret = ret < -1000 ? -1000 : ret;
  return ret;
}

short WC_chassis_mcu::getMotorSpeed(float speed) {
  // transfer real speed to motor comond
  short ret = (short)(speed / (Dia_B_ * M_PI) * Reduction_ratio_ / full_speed_ * 60.0);
  ret = ret > SPEED_CMD_TH ? SPEED_CMD_TH : ret;
  ret = ret < ((-1) * SPEED_CMD_TH) ? ((-1) * SPEED_CMD_TH) : ret;
  ROS_INFO("cc ret = %d", ret);
  return ret;
}
	
double sign(double t){
  return t >= 0.0 ? (1.0) : (-1.0);
}

void WC_chassis_mcu::setTwoWheelSpeed(float speed_v, float speed_w)  {
//  float angle = 0.0;
//  float speed = 0.0;
  ROS_INFO("[CHASSIS] get speed v = %.2f, w = %.2f", speed_v, speed_w);
  float speed_left = 0.0;
  float speed_right = 0.0;
  short m_speed_left = 0;
  short m_speed_right = 0;
  speed_v = fabs(speed_v) > max_speed_v_ ?  sign(speed_v) * max_speed_v_ : speed_v;
  speed_w = fabs(speed_w) > max_speed_w_ ?  sign(speed_w) * max_speed_w_ : speed_w;
  ROS_INFO("[CHASSIS]  cc get max_speed_v_  = %.2f, max_speed_w_ = %.2f", max_speed_v_, max_speed_w_);
  float delta_speed_v = speed_v - last_speed_v_;
  float delta_speed_w = speed_w - last_speed_w_;  
  // speed_v = fabs(delta_speed_v) > DELTA_SPEED_V_TH ? (last_speed_v_ + sign(delta_speed_v) * DELTA_SPEED_V_TH) : speed_v;  
  
  if(delta_speed_v > 0.0) { 
    float delta_speed_v_acc = fabs(speed_v) < 0.01 ? 0.25 : speed_v_acc_;
    speed_v = delta_speed_v > delta_speed_v_acc ? (last_speed_v_ + delta_speed_v_acc) : speed_v;  
 } else {
    float delta_speed_v_dec = fabs(speed_v) < 0.01 ? speed_v_dec_zero_ : speed_v_dec_;
    speed_v = delta_speed_v < delta_speed_v_dec ? (last_speed_v_ + delta_speed_v_dec) : speed_v;  
  }
/*  if(delta_speed_w > 0.0) { 
     speed_w = delta_speed_w > DELTA_SPEED_W_INC_TH ? (last_speed_w_ + DELTA_SPEED_W_INC_TH) : speed_w;  
   } else {
     speed_w = delta_speed_w < DELTA_SPEED_W_DEC_TH ? (last_speed_w_ + DELTA_SPEED_W_DEC_TH) : speed_w;  
  }
*/
  speed_w = fabs(delta_speed_w) > speed_w_acc_ ? (last_speed_w_ + sign(delta_speed_w) * speed_w_acc_) : speed_w;  

  ROS_INFO("[CHASSIS] set real speed v = %.2f, w = %.2f", speed_v, speed_w);
  // calculate angle and speed
//  CalculateAngleAndSpeed(&angle, &speed, speed_v, speed_w);	  

  if (IsInPlaceRotation(speed_v, speed_w))  {
    speed_right = Axle_ * tan(speed_w * TimeWidth_) / (2 * TimeWidth_); 
    speed_left = -1.0 * speed_right;
/*  if (speed_w > 0) {	//Left
      speed_right = Axle_ * tan(speed_w * TimeWidth_) / (2 * TimeWidth_); 
      speed_left = -1.0 * speed_right;
    } else {
      speed_left = Axle_ * tan(speed_w * TimeWidth_) / (2 * TimeWidth_); 
      speed_right = -1.0 * speed_left;
    }
*/
  } else if (IsStop(speed_v, speed_w)) {
    speed_left = 0.0;
    speed_right = 0.0;
  } else {
    if (speed_w > 0.0) {  //Left
//     speed_left = speed_v - (speed_w * Axle_* H_ ) / (2 * speed_v * TimeWidth_);
      speed_left = speed_v - (Axle_ * tan(speed_w * TimeWidth_)) / (2 * TimeWidth_);
      speed_right = 2 * speed_v - speed_left;
    } else { // right
//     speed_right = speed_v + (speed_w * Axle_* H_ ) / (2 * speed_v * TimeWidth_);
      speed_right = speed_v + (Axle_ * tan(speed_w * TimeWidth_)) / (2 * TimeWidth_);
      speed_left = 2 * speed_v - speed_right;
    }
  }
  ROS_INFO("[CHASSIS] raw speed Left: %.2f, Right: %.2f", speed_left, speed_right);
  m_speed_left = getMotorSpeed(speed_left);
  m_speed_right= getMotorSpeed(speed_right);
  
  ROS_INFO("[CHASSIS] cc set motor cmd Left: %d, Right: %d", m_speed_left, m_speed_right);

  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateTwoWheelSpeed(send, &len, m_speed_left, m_speed_right);

  std::string str = cComm::ByteToHexString(send, len);
  std::cout << "send speed: " << str << std::endl;
  //  std::cout << "send speed: " << str.substr(42, 12) << std::endl;
  double start_time = GetTimeInSeconds();
 if (transfer_) {
    transfer_->Send_data(send, len);
#ifdef DEBUG_ETHERNET
    double send_time = GetTimeInSeconds(); 
    if (send_time - start_time> 0.002)
      ROS_INFO("[CHASSIS] set speed: send time = %lf", send_time - start_time);
#endif
    transfer_->Read_data(rec, rlen, 23, 500);
#ifdef DEBUG_ETHERNET
    double recv_time = GetTimeInSeconds();
    if (recv_time - send_time> 0.002)
      ROS_INFO("[CHASSIS] set speed: recv time = %lf", recv_time - send_time);
#endif
  }
 usleep(1000);
	
  last_speed_v_ = speed_v;
  last_speed_w_ = speed_w;
}


void WC_chassis_mcu::comunication(void) {
  delta_counts_left_ = getLPos();
  usleep(1000);
  delta_counts_right_ = getRPos();
  usleep(1000);
  getUltra();
  usleep(1000);
  getYawAngle(yaw_angle_, pitch_angle_, roll_angle_);
  yaw_angle_ = yaw_angle_ < 0 ? (3600 + yaw_angle_) : yaw_angle_;
  usleep(1000); 
}
