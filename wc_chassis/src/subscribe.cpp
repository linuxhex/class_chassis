/*subscribe.cpp 所有接收topic处理函数
 */

#include "subscribe.h"
#include "init.h"
#include "parameter.h"

/*
 * 导航节点发来的速度
 */
void DoNavigationCallback(const geometry_msgs::Twist& Navigation_msg) {
  timeval tv;
  gettimeofday(&tv, NULL);
  last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

  m_speed_v = Navigation_msg.linear.x;
  m_speed_w = Navigation_msg.angular.z;
}

/*
 * 更新遥控器状态
 */
void RemoteRetCallback(const std_msgs::UInt32& ret) {
    remote_ret_ = ret.data;
    remote_ret_cnt_ = 0;
    GS_INFO("[wc_chassis] get remote ret cmd = %d, state = %d", (remote_ret_ & 0xff), (remote_ret_ >> 8) & 0xff);
}
/*
 * 停车时关闭陀螺仪 仅扫地图使能
 */
void GyroUpdateCallback(const std_msgs::UInt32& state) {
  g_chassis_mcu->gyro_state_ = state.data;
  GS_INFO("[wc_chassis] set gyro state = %d", g_chassis_mcu->gyro_state_);
}

/*
 * 通知下位机，关闭除了充电外的所有继电器
 */
void ShutdownCallback(const std_msgs::UInt32& cmd)
{
    unsigned char shutdown = (unsigned char)cmd.data;
    g_chassis_mcu->setShutdownCmd(shutdown);
    GS_ERROR("[wc_chassis] set shutdown cmd = %d", shutdown);
}
