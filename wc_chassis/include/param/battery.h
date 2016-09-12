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
        int battery_count = -1;
        int display_battery_capacity = 0;
        int sum_battery_capacity = 0;
        double battery_value = 0.0;
        double sum_battery_value = 0.0;
        int battery_level = 3;

    };
}

#endif // BATTERY_H
