#ifndef __ADVERTISE_SERVICE__
#define __ADVERTISE_SERVICE__

#include "init.h"

bool CheckRotate(autoscrubber_services::CheckRotate::Request& req, autoscrubber_services::CheckRotate::Response& res);
bool StopRotate(autoscrubber_services::StopRotate::Request& req, autoscrubber_services::StopRotate::Response& res);
bool StartRotate(autoscrubber_services::StartRotate::Request& req, autoscrubber_services::StartRotate::Response& res);
bool CheckHardware(autoscrubber_services::CheckHardware::Request& req, autoscrubber_services::CheckHardware::Response& res);
bool CloseProtector(autoscrubber_services::CloseProtector::Request& req,autoscrubber_services::CloseProtector::Response& res);
bool CloseUltrasonic(autoscrubber_services::CloseUltrasonic::Request& req,autoscrubber_services::CloseUltrasonic::Response& res);
#endif
