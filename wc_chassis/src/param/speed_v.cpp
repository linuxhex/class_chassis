#include "param/speed_v.h"

Speed_v::Speed_v()
{
    ros::NodeHandle speed_v_nh("~/chassis_param/speed_v");
    speed_v_nh.param("max", this->max, static_cast<double>(0.6));//最大速度
    speed_v_nh.param("acc", this->acc, static_cast<double>(0.025));//速度加速度
    speed_v_nh.param("dec", this->dec, static_cast<double>(-0.12));//速度减速度
    speed_v_nh.param("dec_to_zero", this->dec_to_zero, static_cast<double>(-0.12)); //减到０的减速读
    speed_v_nh.param("full",this->full,static_cast<double>(3.0)); //电机满转速度
    speed_v_nh.param("remote_level", this->remote_level, 0);//遥控器控制速度
}

Speed_v::~Speed_v(){}
