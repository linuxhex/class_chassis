#ifndef SPEED_V
#define SPEED_V
#include <ros/ros.h>

class Speed_v {
 public:
      Speed_v();
     ~Speed_v();

      double acc;
      double max;
      double dec;
      double dec_to_zero;
      int remote_level;
      float m_speed_v;
      double speed_ratio      = 1.0;

};

#endif
