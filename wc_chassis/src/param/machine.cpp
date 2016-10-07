#include "param/machine.h"

Machine::Machine()
{
    ros::NodeHandle machine_nh("~/device/machine");
    machine_nh.param("F_DIA", this->f_dia, static_cast<double>(0.125));	// diameter of front wheel
    machine_nh.param("B_DIA", this->b_dia, static_cast<double>(0.125));
    machine_nh.param("AXLE", this->axle, static_cast<double>(0.383));		// length bettween two wheels
    machine_nh.param("COUNTS", this->counts, 12);//霍尔数
    machine_nh.param("REDUCTION_RATIO", this->reduction_ratio, static_cast<double>(30.0));//减速比
    machine_nh.param("delta_counts_th",this->delta_counts_th,800); //码盘防抖动阈值
    machine_nh.param("braker_delay_time",braker_delay_time,static_cast<double>(1.5));//急停等待时间
    machine_nh.param("MAX_RPM",this->max_rpm,3000); //电机满转速度

}
Machine::~Machine(){}
