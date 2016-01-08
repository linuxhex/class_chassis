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

#define MOTOR_SPEED_RATIO	15671
#define SPEED_TH	        (MOTOR_SPEED_RATIO)	
#define SPEED_V_TH		(0.6)
#define SPEED_W_TH		(0.7)
#define DELTA_SPEED_V_INC_TH  (0.025)
#define DELTA_SPEED_V_DEC_TH  (-0.12)
#define DELTA_SPEED_W_TH	    (0.08) 
#define DT (0.1)

extern double ACC_LIM_TH;
extern float g_spe;
extern float g_angle;
extern int current_v_index;
extern int current_w_index;
extern float g_speed_v[3];
extern float g_speed_w[3];
extern pthread_mutex_t speed_mutex;
const float  H = 0.92;
float current_v = 0.0;
float current_theta = 0.0;

WC_chassis_mcu::WC_chassis_mcu()
  : transfer_(0), H_(0.5), Dia_F_(0.2), Dia_B_(0.2), Axle_(0.6), Counts_(4000),
    odom_x_(0.0), odom_y_(0.0), odom_a_(0.0), odom_a_gyro_(0.0),
    delta_counts_left_(0), delta_counts_right_(0),
    yaw_angle_(0),
    last_speed_v_(0.0), last_speed_w_(0.0),
    counts_left_(0), counts_right_(0), first_odo_(true),
    reduction_ratio_(1), direction(0), speed_v_(0), speed_w_(0) {
  memset(send_, 0, 10);
  memset(rec_, 0, 20);
}

WC_chassis_mcu::~WC_chassis_mcu() { }

void WC_chassis_mcu::Init(const std::string& host_name, const std::string& port, float H, float Dia_F, float Dia_B, float Axle, float TimeWidth, int Counts) {
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
  if ((TimeWidth > 0) && (TimeWidth < 0.2)) {
    TimeWidth_ = TimeWidth;
  } else {
    std::cout << "TimeWidth err value:" <<TimeWidth<< std::endl;
  }
}

bool WC_chassis_mcu::is_Auto() {
  return true;
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
  double l_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_left_ * M_PI) / (Counts_ * reduction_ratio_);  // 200000;  // 81920
  double r_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_right_ * M_PI) / (Counts_ * reduction_ratio_);  // 200000;  // 81920

  double dx = (r_wheel_pos + l_wheel_pos) * 0.5;
  double da = (r_wheel_pos - l_wheel_pos) / Axle_;

  if ((fabs(t) > 10e-3) && (fabs(t) < 10e3)) {
    v = dx / t;
    w = da / t;
  } else {
    v = 0;
    w = 0;
  }
  ROS_INFO("get v = %lf, w = %lf; set v = %lf, w = %lf",v, w, speed_v_, speed_w_);
  return true;
}

bool WC_chassis_mcu::getOdo(double &x, double &y, double &a) {
  comunication();

  std::cout << "left: " << counts_left_ << " right: " << counts_right_ << " dleft: " << delta_counts_left_ << " dright: " << delta_counts_right_ << " angle: " << yaw_angle_  << std::endl;

  if (first_odo_) {
    odom_x_ = 0;
    odom_y_ = 0;
    odom_a_ = 0;
    odom_a_gyro_ = 0;

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
 
  int delta_counts_left = (counts_left_ - last_counts_left_);
  int delta_counts_right = (counts_right_ - last_counts_right_);

  if (delta_counts_left > 800) {
    delta_counts_left -= 4294967296;
  } else if (delta_counts_left < -2000) {
    delta_counts_left += 4294967296;
  }
  if (delta_counts_right > 800) {
    delta_counts_right -= 4294967296;
  } else if (delta_counts_right < -2000) {
    delta_counts_right += 4294967296;
  }

  // std::cout << "dleft: " << delta_counts_left << " dright: " << delta_counts_right << std::endl;

  if (abs(delta_counts_right) > 800) {
    std::cout << "err delta_counts_right: " << delta_counts_right << std::endl;
  }
  if (abs(delta_counts_left) > 800) {
    std::cout << "err delta_counts_left: " << delta_counts_left << std::endl;
  }

  double l_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_left * M_PI) / (Counts_ * reduction_ratio_);  // 200000;  // 81920
  double r_wheel_pos = static_cast<double>(Dia_B_ * delta_counts_right * M_PI) / (Counts_ * reduction_ratio_);  // 200000;  // 81920

  double dx = (r_wheel_pos + l_wheel_pos) * 0.5;
  double da = (r_wheel_pos - l_wheel_pos) / Axle_ * odom_a_ratio;
//  double da = asin((r_wheel_pos - l_wheel_pos) / Axle_);
  odom_x_ += dx * cos(odom_a_);
  odom_y_ += dx * sin(odom_a_);
//  odom_a_ += da;

  if (!(abs(delta_counts_left) < critical_delta && abs(delta_counts_right) < critical_delta)) {
    double temp_dtheta = yaw_angle_ - last_yaw_angle_;
    if(temp_dtheta > -3500.0 &&  temp_dtheta < 0.0 )
			temp_dtheta += 3600.0;
    else if(temp_dtheta > 3500.0) 
			temp_dtheta = -1.0 * (3600.0 - temp_dtheta);
    odom_a_ += (temp_dtheta / 10.0) / 180.0 * M_PI;
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
	ROS_INFO("yaw angle = %lf", yaw);
  //ROS_INFO("odom_a=%lf, odom_a_gyro=%lf", odom_a_ * 57.3, odom_a_gyro_ * 57.3); 
  x = odom_x_;
  y = odom_y_;
  a = odom_a_;

  last_counts_left_ = counts_left_;
  last_counts_right_ = counts_right_;
  last_yaw_angle_ = yaw_angle_;
  return true;
}

void WC_chassis_mcu::setThaZero(double zero) {
  tha_zero_ = zero;
}

void WC_chassis_mcu::getUltra() {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateRUltra(send, &len);
  if (transfer_) {
    transfer_->Send_data(send, len);
    transfer_->Read_data(rec, rlen, 23, 500);
  }

  // std::string str = cComm::ByteToHexString(send, len);
  // std::cout << "send ultra: " << str << std::endl;

  // std::string str = cComm::ByteToHexString(rec, rlen);
  // std::cout << "recv ultra: " << str << std::endl;
  //  std::cout << "recv right pos:  " << str.substr(30, 12) << std::endl;
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

int WC_chassis_mcu::getYawAngle() {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  createYawAngle(send, &len);
  if (transfer_) {
    transfer_->Send_data(send, len);
    transfer_->Read_data(rec, rlen, 15, 500);
  }

  // std::string str = cComm::ByteToHexString(send, len);
  // std::cout << "send ultra: " << str << std::endl;

  std::string str = cComm::ByteToHexString(rec, rlen);
  std::cout << "recv Yaw angle: " << str << std::endl;
  if (rlen == 15) {
    for (int i = 0; i < rlen; ++i) {
      if (IRQ_CH(rec[i])) {
	  	return getYaw();
      }
    }
  } else {
    // sleep(1);
  }
}

int WC_chassis_mcu::getLPos() {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateRPos(send, &len, 0);
  if (transfer_) {
    transfer_->Send_data(send, len);
    transfer_->Read_data(rec, rlen, 23, 500);
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

  if (transfer_) {
    transfer_->Send_data(send, len);
    transfer_->Read_data(rec, rlen, 23, 500);
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
void WC_chassis_mcu::setDO(U32 usdo) {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateDO(send, &len, 0, usdo);

  if (transfer_) {
    transfer_->Send_data(send, len);
    transfer_->Read_data(rec, rlen, 23, 500);
  }

  // std::string str = cComm::ByteToHexString(send, len);
  // std::cout << "send do: " << str << std::endl;

  usleep(1000);
}

bool WC_chassis_mcu::setAuto(bool is_auto) {
  is_auto_ = is_auto;
}

unsigned int WC_chassis_mcu::getDI() {
  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateRDI(send, &len, 0);

  if (transfer_) {
    transfer_->Send_data(send, len);
    transfer_->Read_data(rec, rlen, 23, 500);
  }
  // std::string str = cComm::ByteToHexString(rec, rlen);
  // std::cout << "getdi pos: " << str << std::endl;

  if (rlen == 23) {
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

void do_auto_acc(float spe, float angle, float last_v, float last_w) {
  int last_v_index = (current_v_index + 3 - 1) % 3;
  int last_w_index = (current_w_index + 3 - 1) % 3;
  int last_second_v_index = (current_v_index + 3 - 2) % 3;
  int last_second_w_index = (current_w_index + 3 - 2) % 3;
  // minimum vel is 0.04, so we use 0.07 here to represent in-place rotation, hope
  // it works
  if (IsInPlaceRotation(last_v, last_w)) {
    if (IsInPlaceRotation(g_speed_v[last_v_index], g_speed_w[last_w_index])) {
      if (angle > current_theta) {
        g_angle = std::min(current_theta + ACC_LIM_TH * DT, static_cast<double>(angle));
      } else {
        g_angle = std::max(current_theta - ACC_LIM_TH * DT, static_cast<double>(angle));
      }
      // 0 speed for in-place rotation, but speed cannot be 0 when angle becomes M_PI_2 or -M_PI_2;
      g_spe = current_v = fabs(fabs(g_angle) - M_PI_2) < 0.001 ? fabs(H * last_w) : 0.0;
    } else {
      g_spe = current_v = 0.0;
    }
  } else if (IsStop(last_v, last_w)) {
    g_spe = spe;
    return;
  } else {
    if (angle > current_theta) {
      g_angle = std::min(current_theta + ACC_LIM_TH * DT, static_cast<double>(angle));
    } else {
      g_angle = std::max(current_theta - ACC_LIM_TH * DT, static_cast<double>(angle));
    }
    if (IsInPlaceRotation(g_speed_v[last_v_index], g_speed_w[last_w_index]) && fabs(angle - current_theta) > 0.1) {
      current_v = 0.0;
      if (!(!IsInPlaceRotation(g_speed_v[last_v_index], g_speed_w[last_w_index]) &&
            !IsStop(g_speed_v[last_v_index], g_speed_w[last_w_index]) &&
            IsInPlaceRotation(g_speed_v[last_second_v_index], g_speed_w[last_second_w_index]))) {
        g_angle = current_theta;
      }
    } else {
      double ACC_V = 0.15;
      if (fabs(angle - current_theta) > 10e-3) {
        if (fabs(angle - current_theta) > M_PI / 3.0) {
          current_v = 0.0;
        } else {
          ACC_V = fabs(last_v - current_v) * ACC_LIM_TH / fabs(angle - current_theta);
          current_v = current_v + ACC_V * DT;
        }
      } else {
        current_v = last_v;
      }
    }
    g_spe = current_v / cos(g_angle);
  }
  current_theta = g_angle;
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

int WC_chassis_mcu::getMotorSpeed(float speed) {
  int ret = speed * MOTOR_SPEED_RATIO;
//  ret = ret > SPEED_TH ? SPEED_TH : ret;
//  ret = ret < ((-1) * SPEED_TH) ? ((-1) * SPEED_TH) : ret;
  return ret;
}
	
double sign(double t){
  return t >= 0.0 ? (1.0) : (-1.0);
}

void WC_chassis_mcu::setTwoWheelSpeed(float speed_v, float speed_w)  {
//  float angle = 0.0;
//  float speed = 0.0;
  float speed_left = 0.0;
  float speed_right = 0.0;
  short m_speed_left = 0;
  short m_speed_right = 0;
  speed_v = fabs(speed_v) > SPEED_V_TH ?  sign(speed_v) * SPEED_V_TH : speed_v;
  speed_w = fabs(speed_w) > SPEED_W_TH ?  sign(speed_w) * SPEED_W_TH : speed_w;
  
  float delta_speed_v = speed_v - last_speed_v_;
  float delta_speed_w = speed_w - last_speed_w_;  
  // speed_v = fabs(delta_speed_v) > DELTA_SPEED_V_TH ? (last_speed_v_ + sign(delta_speed_v) * DELTA_SPEED_V_TH) : speed_v;  
  
  if(delta_speed_v > 0.0) { 
    speed_v = delta_speed_v > DELTA_SPEED_V_INC_TH ? (last_speed_v_ + DELTA_SPEED_V_INC_TH) : speed_v;  
 } else {
    float delta_speed_v_inc = DELTA_SPEED_V_DEC_TH;
    if(fabs(speed_v) < 0.01) delta_speed_v_inc *= 2.0;
    speed_v = delta_speed_v < delta_speed_v_inc ? (last_speed_v_ + delta_speed_v_inc) : speed_v;  
  }
/*  if(delta_speed_w > 0.0) { 
    speed_w = delta_speed_w > DELTA_SPEED_W_INC_TH ? (last_speed_w_ + DELTA_SPEED_W_INC_TH) : speed_w;  
	} else {
    speed_w = delta_speed_w < DELTA_SPEED_W_DEC_TH ? (last_speed_w_ + DELTA_SPEED_W_DEC_TH) : speed_w;  
  }
*/
  speed_w = fabs(delta_speed_w) > DELTA_SPEED_W_TH ? (last_speed_w_ + sign(delta_speed_w) * DELTA_SPEED_W_TH) : speed_w;  

  // calculate angle and speed
//  CalculateAngleAndSpeed(&angle, &speed, speed_v, speed_w);	  

  if (IsInPlaceRotation(speed_v, speed_w))  {
       speed_right = Axle_ * tan(speed_w * TimeWidth_) / (2 * TimeWidth_); 
       speed_left = -1.0 * speed_right;
/*  	if (speed_w > 0) {	//Left
    	     speed_right = Axle_ * tan(speed_w * TimeWidth_) / (2 * TimeWidth_); 
	     speed_left = -1.0 * speed_right;
  	}else {
	    speed_left = Axle_ * tan(speed_w * TimeWidth_) / (2 * TimeWidth_); 
	    speed_right = -1.0 * speed_left;
  	}
*/
  } else if (IsStop(speed_v, speed_w)) {
    speed_left = 0;
    speed_right = 0;
  } else {
  	if (speed_w > 0) {	//Left
//		speed_left = speed_v - (speed_w * Axle_* H_ ) / (2 * speed_v * TimeWidth_);
	    speed_left = speed_v - (Axle_ * tan(speed_w * TimeWidth_)) / (2 * TimeWidth_);
	    speed_right = 2 * speed_v - speed_left;
  	}else {
//  		speed_right = speed_v + (speed_w * Axle_* H_ ) / (2 * speed_v * TimeWidth_);
	    speed_right = speed_v + (Axle_ * tan(speed_w * TimeWidth_)) / (2 * TimeWidth_);
	    speed_left = 2 * speed_v - speed_right;
  	}
  }
 // ROS_INFO("[CHASSIS] raw speed Left: %.2f, Right: %.2f", speed_left, speed_right);
  m_speed_left = getMotorSpeed(speed_left);
  m_speed_right= getMotorSpeed(speed_right);
  
  ROS_INFO("[CHASSIS] set speed Left: %d, Right: %d", m_speed_left, m_speed_right);

  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  CreateTwoWheelSpeed(send, &len, m_speed_left, m_speed_right);

  // std::string str = cComm::ByteToHexString(send, len);
  // std::cout << "send speed: " << str << std::endl;
  //  std::cout << "send speed: " << str.substr(42, 12) << std::endl;
#ifndef PC_TEST_ONLY
 if (transfer_) {
    transfer_->Send_data(send, len);
    transfer_->Read_data(rec, rlen, 23, 500);
  }
 usleep(1000);
#endif
	
  last_speed_v_ = speed_v;
  last_speed_w_ = speed_w;
}


void WC_chassis_mcu::setSpeed(float speed_v, float speed_w, int planner_type) {
  // calculate angle and speed
  float angle = 0.0;
  float speed = 0.0;
  CalculateAngleAndSpeed(&angle, &speed, speed_v, speed_w);
  // ROS_INFO("[CHASSIS] before auto_acc, speed: %lf, angle: %lf", speed, angle);
  if (planner_type == 0) {
    pthread_mutex_lock(&speed_mutex);
    do_auto_acc(speed, angle, speed_v, speed_w);
    pthread_mutex_unlock(&speed_mutex);
  } else {
    g_spe = speed;
    if (!IsStop(speed_v, speed_w)) {
      g_angle = angle;
    }
  }

  int m_angle = GetCopleyAngle(g_angle);
  int m_spe = GetCopleySpeed(g_spe);

  // ROS_INFO("[CHASSIS] set speed: %d, angle: %d", m_spe, m_angle);

  unsigned char send[1024] = {0};
  int len = 0;

  unsigned char rec[1024] = {0};
  int rlen = 0;

  if (m_spe > 0 && m_spe < 3) m_spe = 3;

  CreateSpeed2(send, &len, m_spe, m_angle);

  // std::string str = cComm::ByteToHexString(send, len);
  // std::cout << "send speed: " << str << std::endl;

  if (transfer_) {
    transfer_->Send_data(send, len);
    transfer_->Read_data(rec, rlen, 23, 500);
  }
  usleep(1000);
}

void WC_chassis_mcu::comunication(void) {
  delta_counts_left_ = getLPos();
  usleep(1000);
  delta_counts_right_ = getRPos();
  usleep(1000);
  getUltra();
  usleep(1000);
  yaw_angle_ = getYawAngle();
  yaw_angle_ = yaw_angle_ < 0 ? (3600 + yaw_angle_) : yaw_angle_;
  usleep(1000); 
}
