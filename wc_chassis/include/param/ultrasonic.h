#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <string>
#include <ros/ros.h>

class Ultrasonicer{
  public:
    Ultrasonicer();
    ~Ultrasonicer();

    std::string *ultrasonic;
    std::string *special_ultrasonic;
    float  min_range = 0.04;  //超声检测的最小距离  默认值0.04
    float  max_range = 1.0;   //超声检测的最大距离  默认值1.0
    int    num = 0; //超声的数量
    double effective_range = 0.4;
    float  special_ultrasonic_offset_dismeter;


};

#endif // ULTRASONIC_H
