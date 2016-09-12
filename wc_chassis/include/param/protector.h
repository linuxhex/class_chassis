#ifndef PROTECTOR_H
#define PROTECTOR_H
#include <ros/ros.h>
#include <vector>

class Protector{
public:

    Protector();
    ~Protector();
    void protectorManage(void);

     int protector_num;
     std::vector<unsigned int> front_protector_list;
     std::vector<unsigned int> rear_protector_list;
     unsigned int protector_hit;
     double protector_hit_time   = 0.0;  //防撞条触发开始时间
     unsigned int protector_bits = 0x00;
     unsigned int protector_value = 0x00 ; //NONE_HIT  防撞条的值．0:表示防撞条没有被触发　!0:表示防撞条被触发

};

#endif // PROTECTOR_H
