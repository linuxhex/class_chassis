/* parameter.cpp  wc_chassis工程所有的全局参数定义
*/

#include"parameter.h"

double ultral_effective_range = 0.4;
double g_odom_x   = 0;
double g_odom_y   = 0;
double g_odom_tha = 0;
double g_odom_v   = 0;
double g_odom_w   = 0;
unsigned int g_dio_count  = 0;
unsigned int g_ret_count  = 0;
unsigned int g_pc_control = 0;
double last_cmd_vel_time  = 0.0;
double max_cmd_interval   = 1.0;
int current_v_index       = 0;
int current_w_index       = 0;
float m_speed_v = 0.0;
float m_speed_w = 0.0;
float g_speed_v[3] = {0.0, 0.0, 0.0};
float g_speed_w[3] = {0.0, 0.0, 0.0};
float g_spe = 0.0;
float g_angle = 0.0;
unsigned int loop_count   = 0;
int rotate_angle = 0;
unsigned int protector_bits=0x00;
unsigned int ultrasonic_bits=0x00;

bool start_rotate_flag    = false;
bool stop_rotate_flag     = true;
bool is_rotate_finished   = false;
bool new_hardware_version_= false;
unsigned int g_di_data_   = 0;
unsigned int g_do_data_   = 0;
unsigned int cur_emergency_status = 1;
double battery_full_level;
double battery_empty_level;
unsigned int remote_ret_     = 0x0a00;
unsigned int charger_monitor_cmd_ = 0;
unsigned int remote_ret_cnt_ = 0;
unsigned char remote_cmd_    = 0;
unsigned short remote_index_ = 0;
pthread_mutex_t speed_mutex;
std::vector<int> g_ultrasonic;
std::vector<unsigned int> front_protector_list;
std::vector<unsigned int> rear_protector_list;
unsigned int connection_status = 1; // mcu ethernet connection status: 0>bad 1>good
int battery_count = -1;
int charge_count = -1;
int display_battery_capacity = 0;
int sum_battery_capacity = 0;
int sum_charge_voltage = 0;

unsigned int protector_hit;

//超声可配的比较
std::string ultrasonic_str[] = {"ultrasonic0","ultrasonic1","ultrasonic2","ultrasonic3","ultrasonic4",
                                "ultrasonic5","ultrasonic6","ultrasonic7","ultrasonic8","ultrasonic9",
                                "ultrasonic10","ultrasonic11","ultrasonic12","ultrasonic13","ultrasonic14"};

//特殊超声下标位置
unsigned char special_ultrasonic_id[15] = {0xff};


float ultrasonic_min_range = 0.04;  //超声检测的最小距离  默认值0.04
float ultrasonic_max_range = 1.0;   //超声检测的最大距离  默认值1.0
//超声接入的数量
int ultrasonic_num=0;

double max_speed_v, max_speed_w; //最大速度＆角速度
double speed_v_acc, speed_v_dec, speed_v_dec_zero; //速度加速度，减速度
double speed_w_acc, speed_w_dec; //角速度加速度，减速度
double full_speed;  //满转速度
int delta_counts_th; //满盘变化阀值（用于码盘防抖动）
int remote_speed_level_ = 0; //遥控器控制速度等级
int battery_level_ = 3;
short charge_voltage_ = 0;
double current_charge_value_ = 0.0;
double charger_low_voltage_ = 0.0;
int remote_id = 1;
int protector_num = 8; //防撞条个数
double inplace_rotating_theta = 0.2; //初始化时旋转角度

std::string hardware_id;
std::string router_ip = std::string("10.7.5.1");
std::string laser_ip = std::string("10.7.5.100");
std::string laser_connection_status = std::string("true");
std::string router_connection_status = std::string("true");
std::thread *checkConnectionThread;
std::string *ultrasonic;
std::string *special_ultrasonic;
float special_ultrasonic_offset_dismeter;
unsigned int protector_value = 0; //防撞条的值．0:表示防撞条没有被触发　!0:表示防撞条被触发
double protector_hit_time = 0;  //防撞条触发开始时间
unsigned char protector_down = 0; //0:表示防撞条没有触发　１:表示防撞条从触发到１ｓ钟时间之内
bool ultrasonic_board_connection = true; //true:超声转接板连接正常
bool on_charge = false; //在充电true 停止充电false
//定时调度任务
boost::asio::io_service *p_io;
boost::asio::deadline_timer *p_update_device_timer;
