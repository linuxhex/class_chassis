#include "data_process.h"
#include"common_function.h"
#include"init.h"
#include"parameter.h"
#include "action.h"




/*
 *继电器状态
*/
void relayStatusManage(void)
{

    relay_status_ = p_chassis_mcu->setChargeCmd(CMD_CHARGER_STATUS);
    if((relay_status_ & 0x03) == STA_CHARGER_ON){
        charger_relay = true;
    } else if((relay_status_ & 0x03) == STA_CHARGER_OFF){
        charger_relay = false;
    }

    if((relay_status_ & (0x03 << 2)) == STA_CTRL_PWR_ON){
        inner_relay = true;
    } else if((relay_status_ & (0x03 << 2)) == STA_CTRL_PWR_OFF){
        inner_relay = false;
    }

    if((relay_status_ & (0x03 << 4)) == STA_MOTOR_PWR_ON){
        outer_relay = true;
    } else if((relay_status_ & (0x03 << 4)) == STA_MOTOR_PWR_OFF){
        outer_relay = false;
    }

    if((relay_status_ & (0x03 << 6)) == STA_CUSTERM_PWR_ON){
        outer_relay = true;
    } else if((relay_status_ & (0x03 << 6)) == STA_CUSTERM_PWR_OFF){
        outer_relay = false;
    }

}
///*
// * 充电电压数据处理
//*/
//void chargeValueManage(void)
//{
//    unsigned int charge_ADC = (g_ultrasonic[22] << 8) | (g_ultrasonic[23] & 0xff);
//    //  double current_charge_voltage = 0.2298 * (charge_ADC - 516);
//    //  double current_charge_voltage = 0.2352 * (charge_ADC - 507);
//    double current_charge_voltage = 0.2398 * (charge_ADC - 512);
//    current_charge_voltage = current_charge_voltage < 1.5 ? 0.0 : current_charge_voltage;
//    current_charge_voltage = current_charge_voltage > 50.0 ? 0.0 : current_charge_voltage;
//    if (charger_cmd_ == CMD_CHARGER_MONITOR && charger_voltage_ < p_charger->low_voltage && current_charge_voltage > p_charger->low_voltage) {
//      timeval tv;
//      gettimeofday(&tv, NULL);
//      charge_on_time_ = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
//     }
//    charger_voltage_ = current_charge_voltage;
//    GS_INFO("[wc_chassis] adc_charge = %d, charger_voltage_: %lf", charge_ADC, charger_voltage_);

//    if(p_charger->full_voltage < battery_value_){
//       p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
//       charger_cmd_    = CMD_CHARGER_OFF;
//       charger_status_ = STA_CHARGER_OFF;
//       return;
//    }

//    if (charger_relay) {
//      charger_status_ = STA_CHARGER_ON;
//      double mileage = (p_chassis_mcu->mileage_right_ + p_chassis_mcu->mileage_left_) / 2;
//      if(charger_voltage_ < p_charger->low_voltage || (fabs(mileage - pre_mileage) >= 0.03)){
//          p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
//          charger_cmd_    = CMD_CHARGER_OFF;
//          charger_status_ = STA_CHARGER_OFF;
//      }
//    } else if (charger_voltage_ > p_charger->low_voltage) {
//      charger_status_ = STA_CHARGER_TOUCHED;
//      if(p_charger != NULL){
//        p_charger->onCharge();
//      }
//    } else {
//      charger_status_ = STA_CHARGER_OFF;
//    }
//    GS_INFO("[wc_chassis] get relay status = 0x%.2x, charger_status_ = %d", relay_status_, charger_status_);

//}

#ifdef TEST_RESTART
void mainBoardTicksManage(void){
    unsigned int cnt_time = p_chassis_mcu->getCntTime();
    std::cout << "[wc_chassis] main_board ticks = " << cnt_time <<std::endl;
}
#endif

/*
 * 获取防撞条＋超声这些设备的状态数据＋充电电压的处理
*/
void updateDeviceStatus(void)
{
  /*超声can连接状处理*/
  if (!old_ultrasonic_) {
     std::istringstream(get_key_value(g_ultrasonic[19], Ultrasonic_board)) >> std::boolalpha >> ultrasonic_board_connection;
  }
  relayStatusManage();
  if(p_charger != NULL){
    p_charger->chargeValueManage();
  }
  protectorManage();
 #ifdef TEST_RESTART
  mainBoardTicksManage();
 #endif
}
