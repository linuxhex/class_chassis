#ifndef __SERVICE__
#define __SERVICE__
#include <ros/ros.h>
#include <autoscrubber_services/StopScrubber.h>
#include <autoscrubber_services/StartRotate.h>
#include <autoscrubber_services/StopRotate.h>
#include <autoscrubber_services/CheckRotate.h>
#include <autoscrubber_services/CheckHardware.h>
#include <autoscrubber_services/ProtectorSwitch.h>
#include <autoscrubber_services/UltrasonicSwitch.h>
#include <autoscrubber_services/StopScrubber.h>
#include <autoscrubber_services/StartRotate.h>
#include <autoscrubber_services/StopRotate.h>
#include <autoscrubber_services/CheckRotate.h>
#include <autoscrubber_services/CheckProtectorStatus.h>
#include <autoscrubber_services/CheckChargeStatus.h>
#include <autoscrubber_services/SetChargeCmd.h>

class Service{
  public:
    Service();
    ~Service();
    bool checkRotate(autoscrubber_services::CheckRotate::Request& req, autoscrubber_services::CheckRotate::Response& res);
    bool stopRotate(autoscrubber_services::StopRotate::Request& req, autoscrubber_services::StopRotate::Response& res);
    bool startRotate(autoscrubber_services::StartRotate::Request& req, autoscrubber_services::StartRotate::Response& res);
    bool checkHardware(autoscrubber_services::CheckHardware::Request& req, autoscrubber_services::CheckHardware::Response& res);
    bool protectorSwitch(autoscrubber_services::ProtectorSwitch::Request& req,autoscrubber_services::ProtectorSwitch::Response& res);
    bool ultrasonicSwitch(autoscrubber_services::UltrasonicSwitch::Request& req,autoscrubber_services::UltrasonicSwitch::Response& res);
    bool checkProtectorStatus(autoscrubber_services::CheckProtectorStatus::Request& req,autoscrubber_services::CheckProtectorStatus::Response& res);
    bool setAutoChargeCmd(autoscrubber_services::SetChargeCmd::Request& req,autoscrubber_services::SetChargeCmd::Response& res);
    bool checkAutoChargeStatus(autoscrubber_services::CheckChargeStatus::Request& req,autoscrubber_services::CheckChargeStatus::Response& res);
    bool testGoLine(autoscrubber_services::TestGoLine::Request& req,autoscrubber_services::TestGoLine::Response& res);
    bool stopGoLine(autoscrubber_services::StopGoLine::Request& req,autoscrubber_services::StopGoLine::Response& res);
    bool checkGoLine(autoscrubber_services::CheckGoLine::Request& req,autoscrubber_services::CheckGoLine::Response& res);
    bool setBrakerDown(autoscrubber_services::BrakerDown::Request& req,autoscrubber_services::BrakerDown::Response& res);

private:
    ros::ServiceServer start_rotate_srv;
    ros::ServiceServer stop_rotate_srv;
    ros::ServiceServer check_rotate_srv;
    ros::ServiceServer check_hardware_srv;
    ros::ServiceServer protector_switch_srv;
    ros::ServiceServer ultrasonic_switch_srv;
    ros::ServiceServer check_protector_status_srv;
    ros::ServiceServer auto_charge_cmd_srv;
    ros::ServiceServer check_auto_charge_status_srv;
    ros::ServiceServer test_go_line_srv;
    ros::ServiceServer stop_go_line_srv;
    ros::ServiceServer check_go_line_srv;
    ros::ServiceServer down_braker_srv;
};

#endif
