#ifndef __SUBSCRIBE__
#define __SUBSCRIBE__

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>

class Subscribe{

public:
    Subscribe();
    ~Subscribe();
    void doNavigationCallback(const geometry_msgs::Twist::ConstPtr& Navigation_msg);
    void remoteRetCallback(const std_msgs::UInt32::ConstPtr& ret);
    void gyroUpdateCallback(const std_msgs::UInt32::ConstPtr& state);
    void shutdownCallback(const std_msgs::UInt32::ConstPtr& cmd);

private:
    ros::Subscriber    navi_sub;
    ros::Subscriber    remote_ret_sub;
    ros::Subscriber    gyro_update_state_sub;
    ros::Subscriber    shutdown_sub;
};

#endif
