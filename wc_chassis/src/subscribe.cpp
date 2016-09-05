/*subscribe.cpp 所有接收topic处理函数
 */

#include "subscribe.h"
#include "init.h"
#include "parameter.h"

Subscribe::Subscribe(){

    this->navi_sub              = p_n->subscribe<geometry_msgs::Twist>("cmd_vel", 10, boost::bind(&Subscribe::doNavigationCallback, this, _1));
    this->remote_ret_sub        = p_device_nh->subscribe<std_msgs::UInt32>("/device/remote_ret", 10, boost::bind(&Subscribe::remoteRetCallback, this, _1));
    this->gyro_update_state_sub = p_n->subscribe<std_msgs::UInt32>("/gyro_update_state", 10, boost::bind(&Subscribe::gyroUpdateCallback, this, _1));
    this->shutdown_sub          = p_n->subscribe<std_msgs::UInt32>("/device/shutdown", 10, boost::bind(&Subscribe::shutdownCallback, this, _1));
}

Subscribe::~Subscribe(){}
/*
 * 导航节点发来的速度
 */
void Subscribe::doNavigationCallback(const geometry_msgs::Twist::ConstPtr& Navigation_msg) {

  timeval tv;
  gettimeofday(&tv, NULL);
  last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

  p_speed_v->m_speed_v = Navigation_msg->linear.x;
  p_speed_w->m_speed_w = Navigation_msg->angular.z;
}

/*
 * 更新遥控器状态
 */
void Subscribe::remoteRetCallback(const std_msgs::UInt32::ConstPtr& ret) {
    remote_ret_ = ret->data;
    remote_ret_cnt_ = 0;
    GS_INFO("[wc_chassis] get remote ret cmd = %d, state = %d", (remote_ret_ & 0xff), (remote_ret_ >> 8) & 0xff);
}
/*
 * 停车时关闭陀螺仪 仅扫地图使能
 */
void Subscribe::gyroUpdateCallback(const std_msgs::UInt32::ConstPtr& state) {
  p_chassis_mcu->gyro_state_ = state->data;
  GS_INFO("[wc_chassis] set gyro state = %d", p_chassis_mcu->gyro_state_);
}

/*
 * 通知下位机，关闭除了充电外的所有继电器
 */
void Subscribe::shutdownCallback(const std_msgs::UInt32::ConstPtr& cmd)
{
    unsigned char shutdown = (unsigned char)cmd->data;
    p_chassis_mcu->setShutdownCmd(shutdown);
    GS_ERROR("[wc_chassis] set shutdown cmd = %d", shutdown);
}
