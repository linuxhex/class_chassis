#include "param/network.h"

Network::Network()
{
    ros::NodeHandle network_nh("~/device/network");
    network_nh.param("host_name", this->host_name, std::string("10.7.5.199"));
    network_nh.param("port", this->port, std::string("5000"));
    network_nh.param("router_ip", this->router_ip, std::string("10.7.5.1"));//路由ip
    network_nh.param("laser_ip", this->laser_ip, std::string("10.7.5.100"));//激光ip
    network_nh.param("internet_url",this->internet_url,std::string(""));//外网url用于测试外网状态
}

Network::~Network(){}
