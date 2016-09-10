#ifndef PROTECTOR_H
#define PROTECTOR_H
#include <ros/ros.h>
#include <vector>

class Protector{
public:

    Protector();
    ~Protector();
     int protector_num;
     std::vector<unsigned int> front_protector_list;
     std::vector<unsigned int> rear_protector_list;

};

#endif // PROTECTOR_H
