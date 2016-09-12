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
  if (p_ultrasonic != NULL) {
     std::istringstream(get_key_value(g_ultrasonic[19], Ultrasonic_board)) >> std::boolalpha >> p_ultrasonic->ultrasonic_board_connection;
  }
  relayStatusManage();
  if(p_charger != NULL){
    p_charger->chargeValueManage();
  }
  if(p_protector != NULL){
    p_protector->protectorManage();
  }
 #ifdef TEST_RESTART
  mainBoardTicksManage();
 #endif
}
