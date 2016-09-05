#include "param/charger.h"

Charger::Charger(){

    ros::NodeHandle charger_nh("~/chassis_param/charger");
    charger_nh.param("low_voltage", this->low_voltage, static_cast<double>(24.5));//初始化
    charger_nh.param("full_voltage", this->full_voltage, static_cast<double>(27.5));//初始化旋转速度
    charger_nh.param("delay_time",this->delay_time,30);//充电继电器打开延时时间

}

Charger::~Charger(){}
