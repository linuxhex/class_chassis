/* advertise_service.cpp 所有的 ros service的回调函数
*/

#include "advertise_service.h"
#include "init.h"
#include "parameter.h"
#include "common_function.h"


/*
 *  自动充电 状态查询
 */
bool CheckAutoChargeStatus(autoscrubber_services::CheckChargeStatus::Request& req,
                           autoscrubber_services::CheckChargeStatus::Response& res)
{
    if (charger_monitor_cmd_ && charge_voltage_ > charger_low_voltage_) {
        unsigned char  status = 0;
        g_chassis_mcu->getChargeStatus(status);
        res.charge_status.status = status;
        res.charge_status.value  = charge_voltage_;
        GAUSSIAN_INFO("calling CheckAutoChargeStatus end!!!");
        return true;
    }
    return false;
}

/*
 *  自动充电 控制命令  cmd  4:停止自动充电　　3:开始自动充电
 */
bool SetAutoChargeCmd(autoscrubber_services::SetChargeCmd::Request& req,
                      autoscrubber_services::SetChargeCmd::Response& res)
{
    unsigned char cmd = req.cmd.data;
    if (1 == cmd) {
      cmd = 3;
      on_charge = true;
    } else {
      cmd = 4;
      on_charge = false;
    }
    g_chassis_mcu->setChargeCmd(cmd);

  return true;
}

/*
 *  提供给上层应用，屏蔽特定的防撞条
 */
bool ProtectorSwitch(autoscrubber_services::ProtectorSwitch::Request& req,
    autoscrubber_services::ProtectorSwitch::Response& res) {
  std::string protector_str =  req.protectorStr.data;
  protector_bits = 0x00;  //bit位置　1:屏蔽  0:不屏蔽
  for(unsigned int i = 0; i < protector_str.length(); ++i){
    if(protector_str[i] == '0'){
      protector_bits |= 0x01<<i;
    }
  }
  return true;
}

/*
 *  提供给上层应用，屏蔽特定的超声
 */
bool UltrasonicSwitch(autoscrubber_services::UltrasonicSwitch::Request& req,
    autoscrubber_services::UltrasonicSwitch::Response& res) {
  std::string ultrasonic_str = req.ultrasonicStr.data;
  ultrasonic_bits = 0x00;  //bit位置　1:屏蔽  0：不屏蔽
  for(unsigned int i = 0; i < ultrasonic_str.length(); ++i){
    if(ultrasonic_str[i] == '0'){
      ultrasonic_bits |= 0x01<<i;
    }
  }
  return true;
}

/*
 *  提供给navigation模块，用于在判断遇到障碍物是是否是防撞条触发
 */
bool CheckProtectorStatus(autoscrubber_services::CheckProtectorStatus::Request& req,
                          autoscrubber_services::CheckProtectorStatus::Response& res)
{
    if(protector_value != NONE_HIT){
       res.protector_status.protect_status=true;
    }else{
       res.protector_status.protect_status=false;
    }
    res.protector_status.protect_value = protector_value;
    protector_value = NONE_HIT;
    return true;
}

/*
 * 初始化　开始旋转
 */
bool StartRotate(autoscrubber_services::StartRotate::Request& req,
                 autoscrubber_services::StartRotate::Response& res)
{
  rotate_angle = req.rotateAngle.data;
  start_rotate_flag = true;
  is_rotate_finished = false;
  m_speed_v  = 0.0;
  m_speed_w  = 0.0;
  g_chassis_mcu->acc_odom_theta_ = 0.0;
  return true;
}

/*
 * 初始化　停止旋转
 */
bool StopRotate(autoscrubber_services::StopRotate::Request& req,
                autoscrubber_services::StopRotate::Response& res) {
  start_rotate_flag = false;
  return true;
}

/*
 * 初始化　旋转检测
 */
bool CheckRotate(autoscrubber_services::CheckRotate::Request& req,
                 autoscrubber_services::CheckRotate::Response& res) {
  res.isFinished.data = is_rotate_finished;
  return true;
}

/*
 *硬件模块的状态,service服务函数
 */
bool CheckHardware(autoscrubber_services::CheckHardware::Request& req, autoscrubber_services::CheckHardware::Response& res) {

  int hardware_s = g_ultrasonic[19];

  diagnostic_msgs::DiagnosticStatus hardware_status;
  diagnostic_msgs::KeyValue value;
  hardware_status.name = std::string("hardware_status");
  hardware_status.message = std::string("status_msgs");
  hardware_status.hardware_id = hardware_id;

  value.key = std::string("MCU_connection"); // 0:bad 1:good
  value.value = (connection_status == 1) ? std::string("true") : std::string("false");
  hardware_status.values.push_back(value);

  value.key = std::string("ultrasonic_board");
  value.value = get_key_value(hardware_s, Ultrasonic_board);
  hardware_status.values.push_back(value);

  value.key = std::string("laser_connection");
  value.value = laser_connection_status;
  hardware_status.values.push_back(value);

  value.key = std::string("router_connection");
  value.value = router_connection_status;
  hardware_status.values.push_back(value);

  //超声状态
  for (int i = 0; i < ultrasonic_num; ++i) {
    if (g_ultrasonic[i + 1] == 0xff || !ultrasonic_board_connection) {
      value.key  = ultrasonic_str[i];
      value.value = std::string("false");
      hardware_status.values.push_back(value);
    } else{
      value.key   = ultrasonic_str[i];
      value.value = std::string("true");
      hardware_status.values.push_back(value);
    }
  }
  res.hardwareStatus = hardware_status;
  return true;
}
