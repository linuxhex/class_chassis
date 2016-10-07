#include "param/battery.h"

Param::Battery::Battery()
{
    ros::NodeHandle battery_nh("~/device/battery");
    battery_nh.param("full_level", battery_full_level, static_cast<double>(27.5));
    battery_nh.param("empty_level", battery_empty_level, static_cast<double>(20.0));

}

Param::Battery::~Battery(){}
