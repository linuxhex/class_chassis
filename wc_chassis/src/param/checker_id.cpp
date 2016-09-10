#include "param/checker_id.h"

Checker_id::Checker_id()
{
    ros::NodeHandle checker_id_nh("~/chassis_param/checker_id");
    checker_id_nh.param("hardware_id", hardware_id, std::string("   "));//硬件设备名称
    checker_id_nh.param("device_id", device_id, std::string("   "));//硬件设备名称
}

Checker_id::~Checker_id(){}

