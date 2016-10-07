#ifndef __CHASSIS_TIME_H__
#define __CHASSIS_TIME_H__
#include <ros/ros.h>

class ChassisTime{
  public:
      ChassisTime();
      ~ChassisTime();

      int controller_rate     = 10;
      int sample_rate      = 10;
      double max_cmd_interval = 0.5;
};
#endif // TIME_H
