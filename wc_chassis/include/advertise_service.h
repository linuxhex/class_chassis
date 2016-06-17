#ifndef __ADVERTISE_SERVICE__
#define __ADVERTISE_SERVICE__

#include "init.h"

extern bool CheckRotate(autoscrubber_services::CheckRotate::Request& req, autoscrubber_services::CheckRotate::Response& res);
extern bool StopRotate(autoscrubber_services::StopRotate::Request& req, autoscrubber_services::StopRotate::Response& res);
extern bool StartRotate(autoscrubber_services::StartRotate::Request& req, autoscrubber_services::StartRotate::Response& res);
extern bool CheckHardware(autoscrubber_services::CheckHardware::Request& req, autoscrubber_services::CheckHardware::Response& res);

#endif
