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
     double timeWidth        = 0;
     int  delta_counts_th    = 800;
     double max_cmd_interval = 0.7;
     bool   braker_down = false;
     double braker_start_time = 0.0;
     double braker_delay_time = 0.0;

};

#endif // MACHINE_H
