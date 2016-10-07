#ifndef MACHINE_H
#define MACHINE_H
#include <ros/ros.h>

class Machine{

public:
     Machine();
    ~Machine();

     double f_dia = 0;
     double b_dia = 0;
     double axle  = 0;
     int counts   = 0;
     double reduction_ratio  = 30.0;
     double speed_ratio      = 1.0;
     int  delta_counts_th    = 800;
     bool   braker_down = false;
     double braker_start_time = 0.0;
     double braker_delay_time = 0.0;
     int    max_rpm = 0;

};

#endif // MACHINE_H
