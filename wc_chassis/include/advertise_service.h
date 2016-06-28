#ifndef __ADVERTISE_SERVICE__
#define __ADVERTISE_SERVICE__

#include "init.h"

bool CheckRotate(autoscrubber_services::CheckRotate::Request& req, autoscrubber_services::CheckRotate::Response& res);
bool StopRotate(autoscrubber_services::StopRotate::Request& req, autoscrubber_services::StopRotate::Response& res);
bool StartRotate(autoscrubber_services::StartRotate::Request& req, autoscrubber_services::StartRotate::Response& res);
bool CheckHardware(autoscrubber_services::CheckHardware::Request& req, autoscrubber_services::CheckHardware::Response& res);
bool ProtectorSwitch(autoscrubber_services::ProtectorSwitch::Request& req,autoscrubber_services::ProtectorSwitch::Response& res);
bool UltrasonicSwitch(autoscrubber_services::UltrasonicSwitch::Request& req,autoscrubber_services::UltrasonicSwitch::Response& res);
bool CheckProtectorStatus(autoscrubber_services::CheckProtectorStatus::Request& req,autoscrubber_services::CheckProtectorStatus::Response& res);


#endif
