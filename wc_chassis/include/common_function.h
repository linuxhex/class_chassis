#ifndef COMMON_FUNCTION_H
#define COMMON_FUNCTION_H
#include <string>
#include "parameter.h"
#define VERIFY_REMOTE_KEY
#define VERIFY_REMTOE_ID

#ifdef SETTING_PRIORITY
#include <sched.h>
#endif

#define SETTING_PRIORITY

unsigned int GenerateJSHash(unsigned int);
std::string get_key_value(int, int);
void freeResource(void);
void SetSchedPriority(void);
void ReadConfigFromXMLRPC(XmlRpc::XmlRpcValue& config_xmlrpc, const std::string& full_param_name, std::vector<unsigned int>* config_list);
bool ReadConfigFromParams(std::string param_name, ros::NodeHandle* nh, std::vector<unsigned int>* config_list);
void protectorManage(void);
void chargeValueManage(void);
void updateDeviceStatus(void);
void relayStatusManage(void);
#endif // COMMON_FUNCTION_H
