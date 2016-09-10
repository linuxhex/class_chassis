#include "param/hand_touch.h"

HandToucher::HandToucher()
{
    ros::NodeHandle hand_touch_nh("~/chassis_param/hand_touch");
    hand_touch_nh.param("new_hand_touch", new_hand_touch, false);//新板子手触开关和防撞条共用一个接口

}

HandToucher::~HandToucher(){}
