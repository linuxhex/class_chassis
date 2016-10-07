#include "param/protector.h"
#include "common_function.h"

Protector::Protector()
{
    ros::NodeHandle protector_nh("~/device/protector");
    protector_nh.param("protector_num",protector_num,8);//防撞条使用数量

    // 前面防撞条配置
    if (!ReadConfigFromParams("front_protector", &protector_nh, &front_protector_list)) {
      GS_ERROR("[SERVICEROBOT] read front_protector_list failed");
    }
    // 后面防撞条配置
    if (!ReadConfigFromParams("rear_protector", &protector_nh, &rear_protector_list)) {
      GS_ERROR("[SERVICEROBOT] read rear_protector_list failed");
    }
}


/*
 * 防撞条数据处理
*/
void Protector::protectorManage(void)
{
    if(protector_num <= 0){
        protector_hit = NONE_HIT;
        protector_value = NONE_HIT;
        return;
    }
    // check protector hit
    unsigned int protector_status = g_ultrasonic[0] | protector_bits;
    unsigned int temp_hit = NONE_HIT;
    for (unsigned int i = 0; i < front_protector_list.size(); ++i) {
      if (!(protector_status & (1 << front_protector_list.at(i)))) {
        temp_hit |= FRONT_HIT;
        GS_ERROR("[WC_CHASSIS] front protector bit[%d] hit!!!", front_protector_list.at(i));
        break;
      }
    }
    for (unsigned int i = 0; i < rear_protector_list.size(); ++i) {
     if (!(protector_status & (1 << rear_protector_list.at(i)))) {
        temp_hit |= REAR_HIT;
        GS_ERROR("[WC_CHASSIS] rear protector bit[%d] hit!!!", rear_protector_list.at(i));
        break;
      }
    }
    if (temp_hit != NONE_HIT) {
       timeval tv;
       protector_value |= temp_hit;
       gettimeofday(&tv, NULL);
       protector_hit_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
     }

    timeval tv;
    gettimeofday(&tv, NULL);
    double time_now = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
    //超过１秒，自动清除给导航的状态
    if((protector_value != NONE_HIT) && (time_now - protector_hit_time > p_chassis_time->max_cmd_interval)){
      protector_value = NONE_HIT;
    }

    protector_hit = temp_hit;

}

Protector::~Protector()
{

}

