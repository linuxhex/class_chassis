#include "data_process.h"
#include"common_function.h"
#include"init.h"
#include"parameter.h"
#include "action.h"


/*
 * 防撞条数据处理
*/
void protectorManage(void)
{
    if(protector_num <= 0){
        protector_hit = NONE_HIT;
        protector_value = NONE_HIT;
        return;
    }
    // check protector hit
    unsigned int protector_status = g_ultrasonic[0] | protector_bits;
    unsigned int temp_hit = NONE_HIT;
    for (unsigned int i = 0; i < front_protector_list.size(); ++i) {
      if (!(protector_status & (1 << front_protector_list.at(i)))) {
        temp_hit |= FRONT_HIT;
        GS_ERROR("[WC_CHASSIS] front protector bit[%d] hit!!!", front_protector_list.at(i));
        break;
      }
    }
    for (unsigned int i = 0; i < rear_protector_list.size(); ++i) {
     if (!(protector_status & (1 << rear_protector_list.at(i)))) {
        temp_hit |= REAR_HIT;
        GS_ERROR("[WC_CHASSIS] rear protector bit[%d] hit!!!", rear_protector_list.at(i));
        break;
      }
    }
    if (temp_hit != NONE_HIT) {
       timeval tv;
       protector_value |= temp_hit;
       gettimeofday(&tv, NULL);
       protector_hit_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
     }

    timeval tv;
    gettimeofday(&tv, NULL);
    double time_now = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
    //超过１秒，自动清除给导航的状态
    if((protector_value != NONE_HIT) && (time_now - protector_hit_time > p_machine->max_cmd_interval)){
      protector_value = NONE_HIT;
    }

    protector_hit = temp_hit;

}

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
/*
 * 充电电压数据处理
*/
void chargeValueManage(void)
{
    unsigned int charge_ADC = (g_ultrasonic[22] << 8) | (g_ultrasonic[23] & 0xff);
    //  double current_charge_voltage = 0.2298 * (charge_ADC - 516);
    //  double current_charge_voltage = 0.2352 * (charge_ADC - 507);
    double current_charge_voltage = 0.2398 * (charge_ADC - 512);
    current_charge_voltage = current_charge_voltage < 10.0 ? 0.0 : current_charge_voltage;
    current_charge_voltage = current_charge_voltage > 50.0 ? 0.0 : current_charge_voltage;
    if (charger_cmd_ == CMD_CHARGER_MONITOR && charger_voltage_ < charger_low_voltage_ && current_charge_voltage > charger_low_voltage_) {
      timeval tv;
      gettimeofday(&tv, NULL);
      charge_on_time_ = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
     }
    charger_voltage_ = current_charge_voltage;
    GS_INFO("[wc_chassis] adc_charge = %d, charger_voltage_: %lf", charge_ADC, charger_voltage_);

    if(charger_full_voltage_ < battery_value_){
       p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
       charger_cmd_    = CMD_CHARGER_OFF;
       charger_status_ = STA_CHARGER_OFF;
       return;
    }

    if (charger_relay) {
      charger_status_ = STA_CHARGER_ON;
      double mileage = (p_chassis_mcu->mileage_right_ + p_chassis_mcu->mileage_left_) / 2;
      if(charger_voltage_ < charger_low_voltage_ || (fabs(mileage - pre_mileage) >= 0.05)){
          p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
          charger_cmd_    = CMD_CHARGER_OFF;
          charger_status_ = STA_CHARGER_OFF;
      }
    } else if (charger_voltage_ > charger_low_voltage_) {
      charger_status_ = STA_CHARGER_TOUCHED;
      onCharge();
    } else {
      charger_status_ = STA_CHARGER_OFF;
    }
    GS_INFO("[wc_chassis] get relay status = 0x%.2x, charger_status_ = %d", relay_status_, charger_status_);

}

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
  chargeValueManage();
  protectorManage();
 #ifdef TEST_RESTART
  mainBoardTicksManage();
 #endif
}
