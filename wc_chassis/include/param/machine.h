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

};

#endif // MACHINE_H
