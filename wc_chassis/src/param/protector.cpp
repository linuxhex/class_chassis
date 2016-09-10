#include "param/protector.h"
#include "common_function.h"

Protector::Protector()
{
    ros::NodeHandle protector_nh("~/chassis_param/protector");
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

Protector::~Protector()
{

}

