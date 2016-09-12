#ifndef CHARGER_H
#define CHARGER_H
#include <ros/ros.h>

class Charger
{
public:
    Charger();
    ~Charger();
    void onCharge(void);
    void chargeValueManage(void);


    double low_voltage  = 0.0;
    double full_voltage = 27.0;
    int delay_time = 30; //充电继电器打开延时时间，默认值30秒

};


#endif // CHARGER_H
