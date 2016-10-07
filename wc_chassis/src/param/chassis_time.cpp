#include "chassis_time.h"

ChassisTime::ChassisTime()
{
   ros::NodeHandle time_nh("~/strategy/chassis/time");
   time_nh.param("controller_rate", this->controller_rate, 10);//主循环频率
   time_nh.param("sample_rate", this->sample_rate, 10);//码盘采样频率
   time_nh.param("max_cmd_interval", this->max_cmd_interval, static_cast<double>(0.5));//
}


ChassisTime::~ChassisTime(){}
