#ifndef __ACTION__
#define __ACTION__
#include <ros/ros.h>

 bool DoRotate(ros::Publisher &rotate_finished_pub);
 void DoDIO(void);
 void DoRemoteRet(void);
#endif
