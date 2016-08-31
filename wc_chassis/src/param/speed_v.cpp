#include "param/speed_v.h"

Speed_v::Speed_v(){
    ros::NodeHandle speed_v_nh("~/chassis_param/speed_v");
    speed_v_nh.param("max", max_speed_v, static_cast<double>(0.6));//最大速度
    speed_v_nh.param("acc", speed_v_acc, static_cast<double>(0.025));//速度加速度
    speed_v_nh.param("dec", speed_v_dec, static_cast<double>(-0.12));//速度减速度
    speed_v_nh.param("dec_to_zero", speed_v_dec_zero, static_cast<double>(-0.12)); //减到０的减速读
    speed_v_nh.param("full",full_speed,static_cast<double>(3.0)); //电机满转速度
    speed_v_nh.param("remote_level", remote_speed_level_, 0);//遥控器控制速度
}

Speed_v::~Speed_v(){

}
