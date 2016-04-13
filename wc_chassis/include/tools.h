#ifndef __TOOLS__
#define __TOOLS__
#include "wc_chassis_mcu.h"  // NOLINT
extern unsigned int remote_ret_;
extern unsigned int remote_ret_cnt_;


extern void RemoteRetCallback(const std_msgs::UInt32& ret);
extern void DoRemoteRet(WC_chassis_mcu g_chassis_mcu);
#endif
