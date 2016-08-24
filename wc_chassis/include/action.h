#ifndef __ACTION__
#define __ACTION__
#include <ros/ros.h>

 bool DoRotate(ros::Publisher &rotate_finished_pub);
 void DoDIO(ros::Publisher going_back_pub);
 void DoRemoteRet(void);
 void onCharge(void);
#endif
