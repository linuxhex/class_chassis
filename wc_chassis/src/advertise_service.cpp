/* advertise_service.cpp 所有的 ros service的回调函数
*/

#include"advertise_service.h"
#include"init.h"
#include "parameter.h"


bool CloseProtector(autoscrubber_services::CloseProtector::Request& req,
                    autoscrubber_services::CloseProtector::Response& res){
    protector_bits = req.protectorBits.data;
    return true;
}

bool CloseUltrasonic(autoscrubber_services::CloseUltrasonic::Request& req,
                    autoscrubber_services::CloseUltrasonic::Response& res){
    ultrasonic_bits = req.ultrasonicBits.data;
    return true;
}


bool StartRotate(autoscrubber_services::StartRotate::Request& req,
                 autoscrubber_services::StartRotate::Response& res) {
  rotate_angle = req.rotateAngle.data;
  start_rotate_flag = true;
  is_rotate_finished = false;
  g_chassis_mcu->acc_odom_theta_ = 0.0;
  return true;
}

bool StopRotate(autoscrubber_services::StopRotate::Request& req,
                autoscrubber_services::StopRotate::Response& res) {
  start_rotate_flag = false;
  return true;
}

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
