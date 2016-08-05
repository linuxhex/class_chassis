#include"schedule.h"
#include"common_function.h"
/*
 * update status about device: charge voltage; protector status ...
 * TODO(cmn): add protector and other status those should be updated on 10Hz
 */
void updateDeviceStatus(const boost::system::error_code& ) {

  /*超声can连接状处理*/
  std::istringstream(get_key_value(g_ultrasonic[19], Ultrasonic_board)) >> std::boolalpha >> ultrasonic_board_connection;

  if(on_charge){
      unsigned int charge_ADC = (g_ultrasonic[22] << 8) | (g_ultrasonic[23] & 0xff);
    //  double charge_value = 0.2298 * (charge_ADC - 516);
    //  double charge_value = 0.2352 * (charge_ADC - 507);
      double charge_value = 0.2330 * (charge_ADC - 509);
      charge_value = charge_value < 0.0 ? 0.0 : charge_value;
      current_charge_value_= charge_value;
      short current_charge_voltage = static_cast <short>(charge_value * 10.0);
      current_charge_voltage = current_charge_voltage < 100 ? 0 : current_charge_voltage;
      current_charge_voltage = current_charge_voltage > 500 ? 0 : current_charge_voltage;
      charge_voltage_ = current_charge_voltage;
      if (charger_monitor_cmd_ && charge_value > charger_low_voltage_) {
        charger_monitor_cmd_ = 0;
        timeval tv;
        gettimeofday(&tv, NULL);
      }

      GAUSSIAN_INFO("[wc_chassis] adc_charge = %d, current_charge_value: %lf, current_charge_voltage: %d, charge_voltage: %d",
                     charge_ADC, charge_value, current_charge_voltage, charge_voltage_);
  }


  // check protector hit
  if(protector_num <= 0){
      return;
  }

  unsigned int status = g_ultrasonic[0] | protector_bits;
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
