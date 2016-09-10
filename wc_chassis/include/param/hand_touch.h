#ifndef HAND_TOUCH_H
#define HAND_TOUCH_H
#include <ros/ros.h>

class HandToucher
{
   public:
       HandToucher();
       ~HandToucher();

       bool new_hand_touch = false;
};
#endif // HAND_TOUCH_H
