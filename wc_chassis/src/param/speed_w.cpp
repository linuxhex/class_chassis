#include "param/speed_w.h"

Speed_w::Speed_w(){
    ros::NodeHandle speed_w_nh("~/chassis_param/speed_w");
    speed_w_nh.param("max", this->max, static_cast<double>(0.6));//最大角速度
    speed_w_nh.param("speed_w_acc", this->acc, static_cast<double>(0.25));//角速度加速度
    speed_w_nh.param("speed_w_dec", this->dec, static_cast<double>(-0.25));//角速度减速度
    speed_w_nh.param("inplace_rotating_theta", this->inplace_rotating_theta, static_cast<double>(0.2));//初始化旋转速度

    this->m_speed_w = 0.0;

}

Speed_w::~Speed_w(){}
