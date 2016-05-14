#include"parameter.h"

double ACC_LIM_TH = 3.0 / 2.0 * M_PI;
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
unsigned int rotate_angle = 0;
bool start_rotate_flag    = false;
bool stop_rotate_flag     = true;
bool is_rotate_finished   = false;
unsigned int g_di_data_   = 0;
unsigned int g_do_data_   = 0;
unsigned int cur_emergency_status = 1;
double battery_full_level;
double battery_empty_level;
unsigned int remote_ret_     = 0x0a00;
unsigned int remote_ret_cnt_ = 0;
unsigned char remote_cmd_    = 0;
unsigned short remote_index_ = 0;
pthread_mutex_t speed_mutex;
std::vector<int> g_ultrasonic;
//超声可配的比较
std::string ultrasonic_str[] = {"ultrasonic0","ultrasonic1","ultrasonic2","ultrasonic3","ultrasonic4",
                                "ultrasonic5","ultrasonic6","ultrasonic7","ultrasonic8","ultrasonic9",
                                "ultrasonic10","ultrasonic11","ultrasonic12","ultrasonic13","ultrasonic14"};

