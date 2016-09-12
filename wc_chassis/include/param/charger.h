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
    unsigned int charger_status = 0x02; //STA_CHARGER_OFF
    double charger_voltage = 0;
    double charge_on_time = 0.0; //检测到充电条电压正常的时间
    unsigned int sleep_cnt = 0; //充电开启线程睡眠时间，当＝０时允许进入线程
    double go_forward_start_time = 0.0; //初始化的时候，检测到正在充电或在充电条上，机器人向前走2s
    unsigned int charger_cmd    =  0x00; //CMD_CHARGER_STATUS;

};


#endif // CHARGER_H
