#ifndef SPEED_W_H
#define SPEED_W_H
#include <ros/ros.h>

class Speed_w{
public:
    Speed_w();
    ~Speed_w();

    double max;
    double acc;
    double dec;
    double inplace_rotating_theta = 0.2;
    float m_speed_w;
};

#endif // SPEED_W_H
