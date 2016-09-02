#ifndef NETWORK_H
#define NETWORK_H
#include <string>
#include <ros/ros.h>

class Network
{
public:
    Network();
    ~Network();

    std::string host_name = std::string("10.7.5.199");
    int port = 5000;
    std::string router_ip = std::string("10.7.5.1");
    std::string laser_ip  = std::string("10.7.5.100");
    std::string internet_url = std::string("www.baidu.com");
};
#endif // NETWORK_H
