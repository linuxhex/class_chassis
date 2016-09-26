#include "param/ultrasonic.h"

Ultrasonicer::Ultrasonicer()
{
    ros::NodeHandle ultrasonic_nh("~/chassis_param/ultrasonic");
    ultrasonic_nh.param("name",ultrasonic,std::string(" "));//配置的超声
    ultrasonic_nh.param("min_range",min_range,static_cast<float>(0.04));//超声最小距离
    ultrasonic_nh.param("max_range",max_range,static_cast<float>(1.0));//超声最大距离
    ultrasonic_nh.param("effective_range", effective_range, static_cast<double>(0.4));//超声有效检测距离
    ultrasonic_nh.param("specialer",special_ultrasonic,std::string(" "));//特殊配置的超声
    ultrasonic_nh.param("specialer_offset_dismeter",special_ultrasonic_offset_dismeter,static_cast<float>(0.15)); //特殊超声偏置距离
    ultrasonic_nh.param("special_ultrasonic_effective_range",special_ultrasonic_effective_range,static_cast<double>(0.4));
}

Ultrasonicer::~Ultrasonicer(){}
