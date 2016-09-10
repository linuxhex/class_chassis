#ifndef BATTERY_H
#define BATTERY_H
#include <ros/ros.h>

namespace Param {

    class Battery{

    public:
        Battery();
        ~Battery();

        double battery_full_level;
        double battery_empty_level;

    };
}

#endif // BATTERY_H
