#ifndef ULTRASONIC_H
#define ULTRASONIC_H
#include <string>
#include <ros/ros.h>

class Ultrasonicer{
  public:
    Ultrasonicer();
    ~Ultrasonicer();

    std::string ultrasonic;
    std::string special_ultrasonic;
    float  min_range = 0.04;  //超声检测的最小距离  默认值0.04
    float  max_range = 1.0;   //超声检测的最大距离  默认值1.0
    int    num = 0; //超声的数量
    double effective_range = 0.4;
    float  special_ultrasonic_offset_dismeter;
    int ultrasonic_num=0;//超声接入的数量
    unsigned int ultrasonic_bits=0x00;

    //超声可配的比较
    std::string ultrasonic_str[15] = {"ultrasonic0","ultrasonic1","ultrasonic2","ultrasonic3","ultrasonic4",
                                    "ultrasonic5","ultrasonic6","ultrasonic7","ultrasonic8","ultrasonic9",
                                    "ultrasonic10","ultrasonic11","ultrasonic12","ultrasonic13","ultrasonic14"};
    //特殊超声下标位置
    unsigned char special_ultrasonic_id[15] = {0xff};
};

#endif // ULTRASONIC_H
