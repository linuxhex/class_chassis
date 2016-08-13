#include"schedule.h"
#include"common_function.h"

/*
 * 防撞条数据处理
*/
void protectorManage(void)
{
    if(protector_num <= 0){
        return;
    }
    // check protector hit
    unsigned int status = g_ultrasonic[0] | protector_bits;
  //  status ^= (0xffff >> (32 - protector_num));
  //  GAUSSIAN_INFO("[WC_CHASSIS] enablel protector [%x]!!!", (0xffff >> (32 - protector_num)));

    unsigned int temp_hit = NONE_HIT;
    for (unsigned int i = 0; i < front_protector_list.size(); ++i) {
      if (!(status & (1 << front_protector_list.at(i)))) {
        temp_hit |= FRONT_HIT;
        GAUSSIAN_ERROR("[WC_CHASSIS] front protector bit[%d] hit!!!", front_protector_list.at(i));
        break;
      }
    }
    for (unsigned int i = 0; i < rear_protector_list.size(); ++i) {
      if (!(status & (1 << rear_protector_list.at(i)))) {
        temp_hit |= REAR_HIT;
        GAUSSIAN_ERROR("[WC_CHASSIS] rear protector bit[%d] hit!!!", rear_protector_list.at(i));
        break;
      }
    }
    if (temp_hit != NONE_HIT) {
      timeval tv;
      protector_value |= temp_hit;
      gettimeofday(&tv, NULL);
      protector_hit_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
    }

    protector_hit = temp_hit;

}
/*
 * 充电电压数据处理
*/
void chargealueManage(void)
{
    unsigned int charge_ADC = (g_ultrasonic[22] << 8) | (g_ultrasonic[23] & 0xff);
    double charge_value = 0.2330 * (charge_ADC - 509);
    charge_value = charge_value < 0.0 ? 0.0 : charge_value;
    current_charge_value_= charge_value;
    short current_charge_voltage = static_cast <short>(charge_value * 10.0);
    current_charge_voltage = current_charge_voltage < 100 ? 0 : current_charge_voltage;
    current_charge_voltage = current_charge_voltage > 500 ? 0 : current_charge_voltage;
    charge_voltage_ = current_charge_voltage;
    GAUSSIAN_INFO("[wc_chassis] adc_charge = %d, current_charge_value: %lf, current_charge_voltage: %d, charge_voltage: %d",
                   charge_ADC, charge_value, current_charge_voltage, charge_voltage_);

}

/*
 * 获取防撞条＋超声这些设备的状态数据
*/
void updateDeviceStatusThread(void)
{
  while(1){
      g_chassis_mcu->getUltra();
      /*超声can连接状处理*/
      if (!old_ultrasonic_) {
         std::istringstream(get_key_value(g_ultrasonic[19], Ultrasonic_board)) >> std::boolalpha >> ultrasonic_board_connection;
      }
      chargealueManage();
      protectorManage();
      usleep(100000);
    }
}

/*
 * ping ip
*/
bool ping(const char* ip) {
    int status, ping_ret;
    status = system((std::string("ping -w 1 ") + std::string(ip)).c_str());
    if (-1 != status) {
        ping_ret = WEXITSTATUS(status);
        if (0 == ping_ret) {
            return true;
        } else if (1 == ping_ret) {
            return false;
        } else {
            GAUSSIAN_ERROR("[chassis] unknown ping status:%d", ping_ret);
            return false;
        }
    } else {
        GAUSSIAN_ERROR("[chassis] ping program can not use");
        return false;
    }
}

/*
 * 检测激光＋路由器网关口连接状态
*/
void checkConnectionHealthThread(void)
{

    while(1){
        if(!ping(laser_ip.c_str()) || !ping(router_ip.c_str())){
            sleep(5);
            if(!ping(laser_ip.c_str())){
                laser_connection_status = std::string("false");
            }
            if(!ping(router_ip.c_str())){
                router_connection_status = std::string("false");
            }
        }else{
          laser_connection_status = std::string("true");
          router_connection_status = std::string("true");
        }
        sleep(30);
    }
}
