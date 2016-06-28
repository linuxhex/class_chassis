/* advertise_service.cpp 所有的 ros service的回调函数
*/

#include"advertise_service.h"
#include"init.h"
#include "parameter.h"

/*
 *  提供给上层应用，屏蔽特定的防撞条
 */
bool CloseProtector(autoscrubber_services::CloseProtector::Request& req,
                    autoscrubber_services::CloseProtector::Response& res){
    protector_bits = req.protectorBits.data;
    return true;
}

/*
 *  提供给上层应用，屏蔽特定的超声
 */
bool CloseUltrasonic(autoscrubber_services::CloseUltrasonic::Request& req,
                    autoscrubber_services::CloseUltrasonic::Response& res){
    ultrasonic_bits = req.ultrasonicBits.data;
    return true;
}

/*
 *  提供给navigation模块，用于在判断遇到障碍物是是否是防撞条触发
 */
bool CheckProtectorStatus(autoscrubber_services::CheckProtectorStatus::Request& req,
                        autoscrubber_services::CheckProtectorStatus::Response& res){

    unsigned int status        = g_ultrasonic[0] | (protector_bits);
    unsigned int protect_value = (status^0xffff) << (32-protector_num);
    if(protect_value != 0){
       res.protector_status.protect_status=true;
    }else{
       res.protector_status.protect_status=false;
    }
    res.protector_status.protect_value = protect_value >> (32-protector_num);
    return true;
}

/*
 * 初始化　开始旋转
 */
bool StartRotate(autoscrubber_services::StartRotate::Request& req,
                 autoscrubber_services::StartRotate::Response& res) {
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

  diagnostic_msgs::DiagnosticStatus hardware_status;
  diagnostic_msgs::KeyValue value;
  hardware_status.name = std::string("hardware_status");
  hardware_status.message = std::string("status_msgs");
  hardware_status.hardware_id = hardware_id;

  value.key = std::string("MCU_connection"); // 0:bad 1:good
  value.value = (connection_status == 1) ? std::string("true") : std::string("false");
  hardware_status.values.push_back(value);

  value.key = std::string("laser_connection");
  value.value = laser_connection_status;
  hardware_status.values.push_back(value);

  value.key = std::string("router_connection");
  value.value = router_connection_status;
  hardware_status.values.push_back(value);

  //超声状态
  for (int i = 0; i < ultrasonic_num; ++i) {
     if (g_ultrasonic[i + 1] == 0xff) {
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
