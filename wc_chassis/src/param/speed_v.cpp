#include "param/speed_v.h"

Speed_v::Speed_v(){

    ros::NodeHandle strategy_nh("~/chassis_param/strategy");
    strategy_nh.param("max", max, static_cast<double>(0.6));
    strategy_nh.param("acc", acc, static_cast<double>(0.025));
    strategy_nh.param("dec", dec, static_cast<double>(-0.12));
    strategy_nh.param("dec_to_zero", dec_to_zero, static_cast<double>(-0.12));
    strategy_nh.param("full",full,static_cast<double>(3.0));
    strategy_nh.param("remote_level", remote_level, 0);
}

Speed_v::~Speed_v(){

}
