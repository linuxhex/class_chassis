#ifndef __PUBLISH__
#define __PUBLISH__
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

 class Publisher{

 public:
     Publisher();
     ~Publisher();

     void publishYaw(void);
     void publishDIO(void);
     void publishOdom(tf::TransformBroadcaster* odom_broadcaster);
     void publishDeviceStatus(void);
     void publishRemoteCmd(unsigned char, unsigned short);
     void publishUltrasonic(ros::Publisher&, const char*, int, unsigned int, double&);
     void publishUltrasonics(void);
     void publishProtectorStatus(void);
     void publishProtectorValue(void);


     ros::Publisher & getGoingBackPub(){
         return going_back_pub;
     }
     ros::Publisher & getRotateFinishedPub(){
         return rotate_finished_pub;
     }


 private:
     ros::Publisher ultrasonic_pub[15];
     ros::Publisher yaw_pub;
     ros::Publisher odom_pub;
     ros::Publisher remote_cmd_pub;
     ros::Publisher device_pub;
     ros::Publisher rotate_finished_pub;
     ros::Publisher protector_pub;
     ros::Publisher going_back_pub;
     ros::Publisher dio_pub;
     ros::Publisher protector_value_pub;

 };
#endif
