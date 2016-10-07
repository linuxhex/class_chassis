/* parameter.cpp  wc_chassis工程所有的全局参数定义
*/

#include "parameter.h"
#include "service.h"
#include "publish.h"
#include "subscribe.h"
#include "wc_chassis_mcu.h"


double g_odom_x   = 0;
double g_odom_y   = 0;
double g_odom_tha = 0;
double g_odom_v   = 0;
double g_odom_w   = 0;
unsigned int g_dio_count  = 0;
unsigned int g_ret_count  = 0;
unsigned int g_pc_control = 0;
double last_cmd_vel_time  = 0.0;

unsigned int loop_count   = 0;
int rotate_angle = 0;

bool start_rotate_flag    = false;
bool stop_rotate_flag     = true;
bool start_goline_flag    = false;
bool stop_goline_flag     = true;
bool is_rotate_finished   = false;
unsigned int g_di_data_   = 0;
unsigned int g_do_data_   = 0;
unsigned int cur_emergency_status = 1;
unsigned int remote_ret_     = 0x0a00;
unsigned int remote_ret_cnt_ = 0;
unsigned char remote_cmd_    = 0;
unsigned int relay_status_ = 0;
unsigned short remote_index_ = 0;
pthread_mutex_t speed_mutex;
std::vector<int> g_ultrasonic(24);

int remote_id = 1;

std::thread *p_checkConnectionThread;

bool charger_relay = false; //充电继电器状态　false:打开，　true:闭合
bool inner_relay   = true;  //内部继电器状态
bool outer_relay   = true;  //外部继电器状态
bool user_relay    = true;  //用户继电器状态
double pre_mileage = 0.0;  //

double start_pose, current_pose;
double distance = 0.0;
double pre_yaw = 0.0;
double sum_yaw = 0.0;

Speed_w   *p_speed_w   = NULL;
Speed_v   *p_speed_v   = NULL;
Publisher *p_publisher = NULL;
Subscribe *p_subscribe = NULL;
Service   *p_service   = NULL;
WC_chassis_mcu  *p_chassis_mcu = NULL;
Machine         *p_machine     = NULL;
Network         *p_network     = NULL;
Charger         *p_charger     = NULL;
Protector       *p_protector   = NULL;
HandToucher     *p_hand_toucher = NULL;
Ultrasonicer    *p_ultrasonic   = NULL;
Param::Battery  *p_battery      = NULL;
Checker_id      *p_checker_id   = NULL;
ChassisTime     *p_chassis_time = NULL;





