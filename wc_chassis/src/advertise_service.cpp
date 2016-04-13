#include"advertise_service.h"
#include"init.h"

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
