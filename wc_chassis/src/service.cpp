/* advertise_service.cpp 所有的 ros service的回调函数
*/
#include "init.h"
#include "parameter.h"
#include "service.h"
#include "action.h"
#include "common_function.h"

Service::Service(){
    start_rotate_srv              = p_device_nh->advertiseService("start_rotate", &Service::startRotate,this);
    stop_rotate_srv               = p_device_nh->advertiseService("stop_rotate", &Service::stopRotate,this);
    check_rotate_srv              = p_device_nh->advertiseService("check_rotate", &Service::checkRotate,this);
    check_hardware_srv            = p_device_nh->advertiseService("check_hardware", &Service::checkHardware,this);
    protector_switch_srv          = p_device_nh->advertiseService("protector_switch",&Service::protectorSwitch,this);
    ultrasonic_switch_srv         = p_device_nh->advertiseService("ultrasonic_switch",&Service::ultrasonicSwitch,this);
    check_protector_status_srv    = p_device_nh->advertiseService("check_protector_status",&Service::checkProtectorStatus,this);
    auto_charge_cmd_srv           = p_device_nh->advertiseService("auto_charge_cmd",&Service::setAutoChargeCmd,this);
    check_auto_charge_status_srv  = p_device_nh->advertiseService("auto_charge_status",&Service::checkAutoChargeStatus,this);
    test_go_line_srv              = p_device_nh->advertiseService("test_go_line",&Service::testGoLine,this);
    stop_go_line_srv              = p_device_nh->advertiseService("stop_go_line",&Service::stopGoLine,this);
    check_go_line_srv             = p_device_nh->advertiseService("check_go_line",&Service::checkGoLine,this);
    down_braker_srv               = p_device_nh->advertiseService("braker_down",&Service::setBrakerDown,this);

}

Service::~Service(){}

/*
 *  自动充电 状态查询
 */
bool Service::checkAutoChargeStatus(autoscrubber_services::CheckChargeStatus::Request& req,
                                    autoscrubber_services::CheckChargeStatus::Response& res)
{
    if(p_charger == NULL){
        return false;
    }
    res.charge_status.status = p_charger->charger_status;
    res.charge_status.value  = p_charger->charger_voltage >= p_charger->low_voltage ? (short)(p_charger->charger_voltage * 10) : 0;
    GS_INFO("[CHASSIS] get real charge status = %d, value = %d, raw_voltage = %lf!!!",
                   res.charge_status.status, res.charge_status.value, p_charger->charger_voltage);
    return true;
}

/*
 *  自动充电 控制命令  cmd 　01:开始自动充电  02:停止自动充电
 */
bool Service::setAutoChargeCmd(autoscrubber_services::SetChargeCmd::Request& req,
                               autoscrubber_services::SetChargeCmd::Response& res) {
  if(p_charger == NULL){
      return false;
  }
  unsigned char cmd = req.cmd.data;
  if (cmd == CMD_CHARGER_ON) {
    if(p_charger->charger_status == STA_CHARGER_ON) {
        GS_INFO("[CHASSIS] current charger_status_ = STA_CHARGER_ON || charger_cmd_ == CMD_CHARGER_ON");
        return true;
    }
    // start a thread to handle auto charger
    if(p_charger != NULL){
        p_charger->onCharge();
    }
  } else if (cmd == CMD_CHARGER_OFF) {
    GS_INFO("[CHASSIS] set charger off!!!");
    p_charger->charger_cmd = CMD_CHARGER_OFF;
    p_chassis_mcu->setChargeCmd(CMD_CHARGER_OFF);
  } else if (cmd == CMD_CHARGER_MONITOR) {
    GS_INFO("[CHASSIS] set charger moniter on!!!");
    p_charger->charger_cmd = CMD_CHARGER_MONITOR;
  } else {
    p_charger->charger_cmd = CMD_CHARGER_STATUS;
  }
  return true;
}
/*
 * 急停
 */
bool Service::setBrakerDown(autoscrubber_services::BrakerDown::Request& req,
                            autoscrubber_services::BrakerDown::Response& res) {
    timeval tv;
    p_machine->braker_start_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
    p_machine->braker_down = true;
    return true;
}
/*
 *  提供给上层应用，屏蔽特定的防撞条
 */
bool Service::protectorSwitch(autoscrubber_services::ProtectorSwitch::Request& req,
    autoscrubber_services::ProtectorSwitch::Response& res) {
  if(p_protector == NULL) {
      return false;
  }
  std::string protector_str =  req.protectorStr.data;
  p_protector->protector_bits = 0x00;  //bit位置　1:屏蔽  0:不屏蔽
  for(unsigned int i = 0; i < protector_str.length(); ++i){
    if(protector_str[i] == '0'){
      p_protector->protector_bits |= 0x01<<i;
    }
  }
  return true;
}

/*
 *  提供给上层应用，屏蔽特定的超声
 */
bool Service::ultrasonicSwitch(autoscrubber_services::UltrasonicSwitch::Request& req,
                               autoscrubber_services::UltrasonicSwitch::Response& res) {
  std::string ultrasonic_str = req.ultrasonicStr.data;
  if(p_ultrasonic != NULL){
      p_ultrasonic->ultrasonic_bits = 0x00;  //bit位置　1:屏蔽  0：不屏蔽
      for(unsigned int i = 0; i < ultrasonic_str.length(); ++i){
        if(ultrasonic_str[i] == '0'){
          p_ultrasonic->ultrasonic_bits |= 0x01<<i;
        }
      }
  }
  return true;
}

/*
 *  提供给navigation模块，用于在判断遇到障碍物是是否是防撞条触发
 */
bool Service::checkProtectorStatus(autoscrubber_services::CheckProtectorStatus::Request& req,
                          autoscrubber_services::CheckProtectorStatus::Response& res){
  if(p_protector == NULL){
      return false;
  }
  if(p_protector->protector_value != NONE_HIT){
    res.protector_status.protect_status = true;
    res.protector_status.protect_value  = p_protector->protector_value;
    p_protector->protector_value = NONE_HIT;
  }else{
    res.protector_status.protect_status=false;
    res.protector_status.protect_value = NONE_HIT;
  }
  return true;
}

/*
 * 初始化　开始旋转
 */
bool Service::startRotate(autoscrubber_services::StartRotate::Request& req,
                          autoscrubber_services::StartRotate::Response& res)
{
     rotate_angle = req.rotateAngle.data;
     start_rotate_flag = true;
     stop_rotate_flag = false;
     p_speed_v->m_speed_v  = 0.0;
     p_speed_w->m_speed_w  = 0.0;
     pre_yaw = 0.0;
     sum_yaw = 0.0;
     p_chassis_mcu->acc_odom_theta_ = 0.0;
     //g_chassis_mcu->ReSetOdom();
     return true;
}

/*
  *  测试用走直线
  */
 bool Service::testGoLine(autoscrubber_services::TestGoLine::Request& req,
                          autoscrubber_services::TestGoLine::Response& res) {
       start_pose = sqrt(fabs(g_odom_x*g_odom_x) + fabs(g_odom_y*g_odom_y));
       //g_chassis_mcu->ReSetOdom();
       double dis_offset = 0.03;
       autoscrubber_services::GoLine go_line = req.go_line;
       distance = go_line.distance < dis_offset ? go_line.distance + dis_offset: go_line.distance;
       p_speed_v->m_speed_v = go_line.line_x;
       p_speed_w->m_speed_w = 0.0;
       start_goline_flag = true;
       stop_goline_flag  = false;
       return true;
 }

 /*
  *  测试用走直线
  */
 bool Service::stopGoLine(autoscrubber_services::StopGoLine::Request& req,
                  autoscrubber_services::StopGoLine::Response& res) {
       p_speed_v->m_speed_v = 0.0;
       p_speed_w->m_speed_w = 0.0;
       start_goline_flag = false;
       stop_goline_flag  = true;
       return true;
 }

 /*
  * 初始化　旋转检测
  */
 bool Service::checkGoLine(autoscrubber_services::CheckGoLine::Request& req,
                  autoscrubber_services::CheckGoLine::Response& res) {
   res.isFinished.data = stop_goline_flag;
   return true;
 }


/*
 * 初始化　停止旋转
 */
bool Service::stopRotate(autoscrubber_services::StopRotate::Request& req,
                autoscrubber_services::StopRotate::Response& res) {
  start_rotate_flag = false;
  return true;
}

/*
 * 初始化　旋转检测
 */
bool Service::checkRotate(autoscrubber_services::CheckRotate::Request& req,
                 autoscrubber_services::CheckRotate::Response& res) {
  res.isFinished.data = is_rotate_finished;
  return true;
}

/*
 *硬件模块的状态,service服务函数
 */
bool Service::checkHardware(autoscrubber_services::CheckHardware::Request& req, autoscrubber_services::CheckHardware::Response& res) {

  int hardware_s = g_ultrasonic[19];

  diagnostic_msgs::DiagnosticStatus hardware_status;
  diagnostic_msgs::KeyValue value;
  hardware_status.name = std::string("hardware_status");
  hardware_status.message = std::string("status_msgs");
  hardware_status.hardware_id = p_checker_id->hardware_id;

  value.key = std::string("MCU_connection"); // 0:bad 1:good
  value.value = p_network->socket_connection_status ? std::string("true") : std::string("false");
  hardware_status.values.push_back(value);

  value.key = std::string("ultrasonic_board");
  value.value = get_key_value(hardware_s, Ultrasonic_board);
  hardware_status.values.push_back(value);

  value.key = std::string("power_board");
  value.value = get_key_value(hardware_s, Power_board);
  hardware_status.values.push_back(value);

  value.key = std::string("gyro_board");
  value.value = get_key_value(hardware_s, Gyro_board);
  hardware_status.values.push_back(value);

  if(p_network != NULL){
      value.key = std::string("laser_connection");
      value.value = p_network->laser_connection_status ? std::string("true") : std::string("false");
      hardware_status.values.push_back(value);

      value.key = std::string("router_connection");
      value.value = p_network->router_connection_status ? std::string("true") : std::string("false");
      hardware_status.values.push_back(value);

      value.key = std::string("internet_connection");
      value.value = p_network->internet_connection_status ? std::string("true") : std::string("false");
      hardware_status.values.push_back(value);
  }
  value.key = std::string("inner_relay_status");
  value.value = inner_relay ? std::string("true") : std::string("false");
  hardware_status.values.push_back(value);

  value.key = std::string("outer_relay_status");
  value.value = outer_relay ? std::string("true") : std::string("false");
  hardware_status.values.push_back(value);

  value.key = std::string("charger_relay_status");
  value.value = charger_relay ? std::string("true") : std::string("false");
  hardware_status.values.push_back(value);

  value.key = std::string("user_relay_status");
  value.value = user_relay ? std::string("true") : std::string("false");
  hardware_status.values.push_back(value);

  //超声状态
  if(p_ultrasonic != NULL){
      for (int i = 0; i < p_ultrasonic->ultrasonic_num; ++i) {
        if (g_ultrasonic[i + 1] == 0xff || g_ultrasonic[i + 1] == 0x00 || !p_ultrasonic->ultrasonic_board_connection) {
          value.key  = p_ultrasonic->ultrasonic_str[i];
          value.value = std::string("false");
          hardware_status.values.push_back(value);
        } else{
          value.key   = p_ultrasonic->ultrasonic_str[i];
          value.value = std::string("true");
          hardware_status.values.push_back(value);
        }
      }
  }
  res.hardwareStatus = hardware_status;
  return true;
}
