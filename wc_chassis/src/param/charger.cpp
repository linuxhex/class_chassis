#include "param/charger.h"
#include "parameter.h"

Charger::Charger(){

    ros::NodeHandle charger_nh("~/chassis_param/charger");
    charger_nh.param("low_voltage", this->low_voltage, static_cast<double>(24.5));//初始化
    charger_nh.param("full_voltage", this->full_voltage, static_cast<double>(27.5));//初始化旋转速度
    charger_nh.param("delay_time",this->delay_time,30);//充电继电器打开延时时间

}

void Charger::onCharge(void){
    if(sleep_cnt == 0){
        charger_cmd_ = CMD_CHARGER_ON;
        std::thread([&](){
          GS_INFO("[CHASSIS] start to check charger volatage = %d", charger_voltage_);
          sleep(3);
          int check_charger_cnt = 0;
          while(++sleep_cnt  < 60 && charger_cmd_ == CMD_CHARGER_ON) {
           GS_INFO("[CHASSIS] checking charger volatage = %d", charger_voltage_);
           if (charger_voltage_ >= low_voltage) {
             ++check_charger_cnt;
           } else {
             check_charger_cnt = 0;
           }
           if (check_charger_cnt > delay_time && charger_cmd_ == CMD_CHARGER_ON) {
             GS_INFO("[CHASSIS] check charger voltage normal > 30s, set charger relay on!!!");
             p_chassis_mcu->setChargeCmd(CMD_CHARGER_ON);
             pre_mileage = (p_chassis_mcu->mileage_right_ + p_chassis_mcu->mileage_left_) / 2;
             break;
           }
            sleep(1);
          }
          sleep_cnt = 0; //允许下次线程能够进入
        }).detach();
    }

}

/*
 * 充电电压数据处理
*/
void Charger::chargeValueManage(void)
{
    unsigned int charge_ADC = (g_ultrasonic[22] << 8) | (g_ultrasonic[23] & 0xff);
    //  double current_charge_voltage = 0.2298 * (charge_ADC - 516);
    //  double current_charge_voltage = 0.2352 * (charge_ADC - 507);
    double current_charge_voltage = 0.2398 * (charge_ADC - 512);
    current_charge_voltage = current_charge_voltage < 1.5 ? 0.0 : current_charge_voltage;
    current_charge_voltage = current_charge_voltage > 50.0 ? 0.0 : current_charge_voltage;
    if (charger_cmd_ == CMD_CHARGER_MONITOR && charger_voltage_ < low_voltage && current_charge_voltage > low_voltage) {
      timeval tv;
      gettimeofday(&tv, NULL);
      charge_on_time_ = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
     }
    charger_voltage_ = current_charge_voltage;
    GS_INFO("[wc_chassis] adc_charge = %d, charger_voltage_: %lf", charge_ADC, charger_voltage_);

    if(full_voltage < battery_value_){
       p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
       charger_cmd_    = CMD_CHARGER_OFF;
       charger_status_ = STA_CHARGER_OFF;
       return;
    }

    if (charger_relay) {
      charger_status_ = STA_CHARGER_ON;
      double mileage = (p_chassis_mcu->mileage_right_ + p_chassis_mcu->mileage_left_) / 2;
      if(charger_voltage_ < low_voltage || (fabs(mileage - pre_mileage) >= 0.03)){
          p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
          charger_cmd_    = CMD_CHARGER_OFF;
          charger_status_ = STA_CHARGER_OFF;
      }
    } else if (charger_voltage_ > low_voltage) {
      charger_status_ = STA_CHARGER_TOUCHED;
      onCharge();
    } else {
      charger_status_ = STA_CHARGER_OFF;
    }
    GS_INFO("[wc_chassis] get relay status = 0x%.2x, charger_status_ = %d", relay_status_, charger_status_);

}

Charger::~Charger(){}
