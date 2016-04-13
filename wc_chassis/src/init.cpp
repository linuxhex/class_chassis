#include "init.h"
#include "advertise_service.h"
double ACC_LIM_TH = 3.0 / 2.0 * M_PI;
double ultral_effective_range = 0.4;
double g_odom_x   = 0;
double g_odom_y   = 0;
double g_odom_tha = 0;
double g_odom_v = 0;
double g_odom_w = 0;

unsigned int g_dio_count  = 0;
unsigned int g_ret_count  = 0;
unsigned int g_pc_control = 0;

double last_cmd_vel_time = 0.0;
double max_cmd_interval  = 1.0;

int current_v_index = 0;
int current_w_index = 0;
float m_speed_v = 0.0;
float m_speed_w = 0.0;
float g_speed_v[3] = {0.0, 0.0, 0.0};
float g_speed_w[3] = {0.0, 0.0, 0.0};
float g_spe = 0.0;
float g_angle = 0.0;
pthread_mutex_t speed_mutex;

unsigned int loop_count = 0;
unsigned int rotate_angle = 0;

bool start_rotate_flag = false;
bool stop_rotate_flag = true;
bool is_rotate_finished = false;

unsigned int g_di_data_ = 0;
unsigned int g_do_data_ = 0;
unsigned int cur_emergency_status = 1;
double battery_full_level;
double battery_empty_level;

unsigned int remote_ret_ = 0x0a00;
unsigned int remote_ret_cnt_ = 0;
unsigned char remote_cmd_ = 0;
unsigned short remote_index_ = 0;
tf::TransformBroadcaster *odom_broadcaster;
WC_chassis_mcu *g_chassis_mcu;
ros::Rate *p_loop_rate;

std::vector<int> g_ultrasonic;


ros::NodeHandle *p_n;
ros::NodeHandle *p_nh;
ros::NodeHandle *p_device_nh;


bool DoRotate() {
  if (fabs(g_chassis_mcu->acc_odom_theta_) >= fabs(rotate_angle / 180.0 * M_PI * 0.98) ) {
    start_rotate_flag = false;
    is_rotate_finished = true;
    g_chassis_mcu->setTwoWheelSpeed(0.0, 0.0);
//    ROS_INFO("[wc_chassis] rotate finished!");
    timeval tv;
    gettimeofday(&tv, NULL);
    last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
  } else {
    is_rotate_finished = false;
//    ROS_INFO("[wc_chassis] inplace rotation: cur_yaw = %lf, targer_yaw = %d ", g_chassis_mcu->acc_odom_theta_ * 57.3, rotate_angle);
    if (rotate_angle > 0) {
      g_chassis_mcu->setTwoWheelSpeed(0.0, 0.2);
//      ROS_INFO("[wc_chassis] rotate to left");
    } else if (rotate_angle < 0) {
      g_chassis_mcu->setTwoWheelSpeed(0.0, -0.2);
//      ROS_INFO("[wc_chassis] rotate to right");
    } else {
      is_rotate_finished = true;
      start_rotate_flag = false;
      g_chassis_mcu->setTwoWheelSpeed(0.0, 0.0);
    }
  }
  return true;
}

void DoDIO() {
  if ((g_dio_count++ % 2) == 0) {
    g_di_data_ = g_chassis_mcu->doDIO(g_do_data_);
    cur_emergency_status = (g_di_data_ >> (8 + Emergency_stop)) & 0x01;
  }
}

void DoRemoteRet() {
  if (++remote_ret_cnt_ > 15) {
    remote_ret_ &= 0xff00;
  }
  g_chassis_mcu->setRemoteRet(remote_ret_);
}


/***
 * 初始化所有的Service和订阅服务
 */
void InitService(){
    p_device_nh->advertiseService("start_rotate", &StartRotate);
    p_device_nh->advertiseService("stop_rotate", &StopRotate);
    p_device_nh->advertiseService("check_rotate", &CheckRotate);
    p_n->subscribe("cmd_vel", 10, DoNavigationCallback);
    p_device_nh->subscribe("/remote_ret", 10, RemoteRetCallback);
    p_n->subscribe("/gyro_update_state", 10, GyroUpdateCallback);
}

/***
 *  用到的所有的参数的初始化
 */
void InitParameter(){

    g_chassis_mcu = new WC_chassis_mcu();
    odom_broadcaster = new tf::TransformBroadcaster();

    double f_dia = 0;
    double b_dia = 0;
    double axle  = 0;
    int counts = 0;
    int reduction_ratio = 0;
    double speed_ratio = 1.0;
    double timeWidth = 0;
    std::string host_name;
    int port;
    std::string str_auto_topic;
    std::string str_odom = "odom";
    p_nh->param("odom", str_odom, str_odom);
    p_nh->param("WC_Auto_topic", str_auto_topic, std::string("WC_AUTO"));
    p_nh->param("max_cmd_interval", max_cmd_interval, 1.0);
    p_nh->param("F_DIA", f_dia, static_cast<double>(0.125));	// diameter of front wheel
    p_nh->param("B_DIA", b_dia, static_cast<double>(0.125));
    p_nh->param("AXLE", axle, static_cast<double>(0.383));		// length bettween two wheels
    p_nh->param("COUNTS", counts, 12);
    p_nh->param("REDUCTION_RATIO", reduction_ratio, 30);
    p_nh->param("SPEED_RATIO", speed_ratio, static_cast<double>(1.0));
    p_nh->param("TimeWidth", timeWidth, static_cast<double>(0.1));
    p_nh->param("ultral_effective_range", ultral_effective_range, static_cast<double>(0.4));
    p_nh->param("host_name", host_name, std::string("192.168.1.199"));
    p_nh->param("port", port, 5000);
    p_nh->param("acc_lim_th", ACC_LIM_TH, 3.0 / 2.0 * M_PI);
    p_nh->param("battery_full_level", battery_full_level, static_cast<double>(27.5));
    p_nh->param("battery_empty_level", battery_empty_level, static_cast<double>(20.0));
    battery_full_level *= 100.0;
    battery_empty_level *= 100.0;

    pthread_mutex_init(&speed_mutex, NULL);
    g_chassis_mcu->Init(host_name, std::to_string(port), 0.975, f_dia, b_dia, axle, timeWidth, counts, reduction_ratio, speed_ratio);

}
/***
 * chassis的初始化
 */
bool InitChassis(int argc, char **argv){
     ros::init(argc, argv, "wc_chassis");
     p_n = new ros::NodeHandle();
     p_nh = new ros::NodeHandle("~");
     p_device_nh = new ros::NodeHandle("device");

     InitService();
     InitParameter();
     p_loop_rate =  new ros::Rate(10);
     return true;
}
