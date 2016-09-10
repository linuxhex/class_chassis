#ifndef CHECKER_ID_H
#define CHECKER_ID_H
#include <ros/ros.h>

class Checker_id
{
public:
    Checker_id();
    ~Checker_id();

    std::string hardware_id;
    std::string device_id;

};



#endif // CHECKER_ID_H
