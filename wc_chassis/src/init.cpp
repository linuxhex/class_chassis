/* init.cpp文件包含了所有wc_chassis工程的初始化
*/

#include "init.h"
#include "subscribe.h"
#include "parameter.h"
#include "common_function.h"
#include "schedule.h"
#include "param/machine.h"

/***ros 相关****/
tf::TransformBroadcaster *p_odom_broadcaster = NULL;
ros::Rate *p_loop_rate = NULL;
ros::NodeHandle *p_n   = NULL;
ros::NodeHandle *p_nh  = NULL;
ros::NodeHandle *p_device_nh =NULL;

/***
 * 初始化所有的Service和订阅服务
 */
void InitService()
{
    p_service   = new Service();
    p_subscribe = new Subscribe();
    p_publisher = new Publisher();
    GS_INFO("[wc_chassis] init service & topic caller completed");
}

/*  ros参数服务器参数的初始化*/
void InitParameter()
{

    p_chassis_mcu      = new WC_chassis_mcu();
    p_odom_broadcaster = new tf::TransformBroadcaster();

    //read device param
    ros::NodeHandle chassis_param_nh("~/device");
    std::string device_params;
    chassis_param_nh.param("devices", device_params, std::string(""));
    std::cout<<"devices"<<" "<<device_params<<std::endl;
    std::stringstream device_ss(device_params);
    std::string device_param;
    while (device_ss >> device_param) {
      if (device_param == "charger") {
          p_charger = new Charger();

      } else if (device_param == "protector") {
          p_protector = new Protector();

      } else if (device_param == "hand_touch") {
          p_hand_toucher = new HandToucher();

      } else if (device_param == "ultrasonic") {
          p_ultrasonic = new Ultrasonicer();

      } else if (device_param == "battery") {
          p_battery = new Param::Battery();

      } else if (device_param == "machine") {
          p_machine = new Machine();

      } else if (device_param == "network") {
          p_network = new Network();

      } else if (device_param == "checker_id") {
          p_checker_id = new Checker_id();

      } /*else if (device_param == "footprint") {
          ros::NodeHandle footprint_nh("~/chassis_param/footprint");

      } else if (device_param == "laser") {
          ros::NodeHandle laser_nh("~/chassis_param/laser");

      }*/
    }

    //read strategy param
    ros::NodeHandle strategy_nh("~/strategy/chassis");
    std::string strategy_params;
    strategy_nh.param("strategies", strategy_params, std::string(""));
    std::cout<<"strategy"<<" "<<strategy_params<<std::endl;
    std::stringstream strategy_ss(strategy_params);
    std::string strategy_param;
    while (strategy_ss >> strategy_param) {
      if (strategy_param == "speed_v") {
          p_speed_v = new Speed_v();
      } else if (strategy_param == "speed_w") {
          p_speed_w = new Speed_w();
      } else if (strategy_param == "time") {
          p_chassis_time = new ChassisTime();
      }
    }

    if(p_chassis_time == NULL){
        std::cout << "[fatal] must configure chassis_time" << std::endl;
        exit(0);
    }
    if(p_speed_v == NULL){
        std::cout << "[fatal] must configure speed_v" << std::endl;
        exit(0);
    }
    if(p_speed_w == NULL){
        std::cout << "[fatal] must configure speed_w" << std::endl;
        exit(0);
    }
    if(p_machine == NULL){
        std::cout << "[fatal] must configure machine" << std::endl;
        exit(0);
    }
    if(p_battery == NULL){
        std::cout << "[fatal] must configure battery" << std::endl;
        exit(0);
    }
    if(p_network == NULL){
        std::cout << "[fatal] must configure network" << std::endl;
        exit(0);
    }


    if(p_charger == NULL){
        std::cout << "[error], not configure auto_charger" << std::endl;
    }
    if(p_protector == NULL){
        std::cout << "[error], not configure protector" << std::endl;
    }
    if(p_hand_toucher == NULL){
        std::cout << "[error], not configure hand_toucher" << std::endl;
    }
    if(p_ultrasonic == NULL){
        std::cout << "[error], not configure ultrasonic" << std::endl;
    }
    if(p_checker_id == NULL){
        std::cout << "[error], not configure checker_id" << std::endl;
    }

    pthread_mutex_init(&speed_mutex, NULL);
    p_chassis_mcu->Init();
    GS_INFO("[wc_chassis] init param completed");
}

/* 设备的初始化*/
void InitDevice(void)
{

#ifdef VERIFY_REMOTE_KEY
  srand((unsigned int)time(NULL));
  unsigned int seed_key = rand();
  unsigned int check_key = GenerateJSHash(seed_key);
  unsigned int verify_key = p_chassis_mcu->checkRemoteVerifyKey(seed_key);
  if (check_key != verify_key) {
    GS_INFO("[wc_chassis] VERIFY_REMOTE_KEY is not correct!!!");
    exit(0);
  }
#endif

#ifdef VERIFY_REMTOE_ID
  std::string str_id;
  if (!gs::file::ReadFile("param_device", str_id)) {
    remote_id = 2;
  } else {
    str_id = str_id.substr(str_id.find(' ') + 1, str_id.size());
      int temp_id = std::atoi(str_id.c_str());
    remote_id = temp_id > 0 && temp_id < 10 ? temp_id : 1;
    std::cout << "remote_id = " << remote_id << std::endl;
  }
#endif
  p_chassis_mcu->setRemoteID((unsigned char)((remote_id & 0x0f) | ((p_speed_v->remote_level & 0x03) << 4) | ((p_battery->battery_level & 0x03) << 6)));
  GS_INFO("[wc_chassis] init device completed");
}

/* 多线程任务初始化 */
void InitSchedule(void)
{
    p_checkConnectionThread    = new std::thread(checkConnectionHealthThread);
    p_checkConnectionThread->detach();
    GS_INFO("[wc_chassis] init schedule completed");
}

/* chassis的初始化*/
bool InitChassis(int argc, char **argv,const char *node_name)
{
   GS_INFO("[wc_chassis] init ros!");
   ros::init(argc, argv, node_name);

   p_n = new ros::NodeHandle();
   p_nh = new ros::NodeHandle("~");
   p_device_nh = new ros::NodeHandle("device");

   InitParameter();

   p_loop_rate =  new ros::Rate(p_chassis_time->controller_rate);

   InitService();
   InitDevice();
   InitSchedule();
   return true;
}
