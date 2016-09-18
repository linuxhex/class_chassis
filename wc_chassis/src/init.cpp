/* init.cpp文件包含了所有wc_chassis工程的初始化
*/

#include "init.h"
#include "advertise_service.h"
#include "subscribe.h"
#include "parameter.h"
#include "common_function.h"
#include "schedule.h"

/***ros 相关****/
tf::TransformBroadcaster *p_odom_broadcaster;
WC_chassis_mcu *g_chassis_mcu;
ros::Rate *p_loop_rate;
ros::NodeHandle *p_n;
ros::NodeHandle *p_nh;
ros::NodeHandle *p_device_nh;

/***ServiceServer***/
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
ros::ServiceServer down_breaker_srv;


/***Subscriber***/
ros::Subscriber    Navi_sub;
ros::Subscriber    remote_ret_sub;
ros::Subscriber    gyro_update_state_sub;
ros::Subscriber    shutdown_sub;
/***
 * 初始化所有的Service和订阅服务
 */
void InitService()
{
    start_rotate_srv              = p_device_nh->advertiseService("start_rotate", &StartRotate);
    stop_rotate_srv               = p_device_nh->advertiseService("stop_rotate", &StopRotate);
    check_rotate_srv              = p_device_nh->advertiseService("check_rotate", &CheckRotate);
    check_hardware_srv            = p_device_nh->advertiseService("check_hardware", &CheckHardware);
    protector_switch_srv          = p_device_nh->advertiseService("protector_switch",&ProtectorSwitch);
    ultrasonic_switch_srv         = p_device_nh->advertiseService("ultrasonic_switch",&UltrasonicSwitch);
    check_protector_status_srv    = p_device_nh->advertiseService("check_protector_status",&CheckProtectorStatus);
    auto_charge_cmd_srv           = p_device_nh->advertiseService("auto_charge_cmd",&SetAutoChargeCmd);
    check_auto_charge_status_srv  = p_device_nh->advertiseService("auto_charge_status",&CheckAutoChargeStatus);
    test_go_line_srv              = p_device_nh->advertiseService("test_go_line",&TestGoLine);
    stop_go_line_srv              = p_device_nh->advertiseService("stop_go_line",&StopGoLine);
    check_go_line_srv             = p_device_nh->advertiseService("check_go_line",&CheckGoLine);


    Navi_sub              = p_n->subscribe("cmd_vel", 10, DoNavigationCallback);
    remote_ret_sub        = p_device_nh->subscribe("/device/remote_ret", 10, RemoteRetCallback);
    gyro_update_state_sub = p_n->subscribe("/gyro_update_state", 10, GyroUpdateCallback);
    shutdown_sub          = p_n->subscribe("/device/shutdown", 10, ShutdownCallback);
    GS_INFO("[wc_chassis] init service & topic caller completed");

}

/*  ros参数服务器参数的初始化*/
void InitParameter()
{

    g_chassis_mcu = new WC_chassis_mcu();
    p_odom_broadcaster = new tf::TransformBroadcaster();
    double f_dia = 0;
    double b_dia = 0;
    double axle  = 0;
    int counts = 0;
    ultrasonic = new std::string();
    special_ultrasonic = new std::string();
    double reduction_ratio = 30.0;
    double speed_ratio = 1.0;
    double timeWidth = 0;
    std::string host_name;
    int port;


    p_nh->param("max_cmd_interval", max_cmd_interval, 1.0);
    p_nh->param("TimeWidth", timeWidth, static_cast<double>(0.1));
    p_nh->param("host_name", host_name, std::string("10.7.5.199"));
    p_nh->param("port", port, 5000);

    p_nh->param("F_DIA", f_dia, static_cast<double>(0.125));	// diameter of front wheel
    p_nh->param("B_DIA", b_dia, static_cast<double>(0.125));
    p_nh->param("AXLE", axle, static_cast<double>(0.383));		// length bettween two wheels
    p_nh->param("COUNTS", counts, 12);//霍尔数
    p_nh->param("REDUCTION_RATIO", reduction_ratio, static_cast<double>(30.0));//减速比
    p_nh->param("SPEED_RATIO", speed_ratio, static_cast<double>(1.0));
    p_nh->param("battery_full_level", battery_full_level, static_cast<double>(27.5));
    p_nh->param("battery_empty_level", battery_empty_level, static_cast<double>(20.0));

    p_nh->param("ultrasonic",*ultrasonic,std::string(" "));//配置的超声
    p_nh->param("ultrasonic_min_range",ultrasonic_min_range,static_cast<float>(0.04));//超声最小距离
    p_nh->param("ultrasonic_max_range",ultrasonic_max_range,static_cast<float>(1.0));//超声最大距离
    p_nh->param("ultral_effective_range", ultral_effective_range, static_cast<double>(0.4));//超声有效检测距离
    p_nh->param("special_ultrasonic",*special_ultrasonic,std::string(" "));//特殊配置的超声
    p_nh->param("special_ultrasonic_offset_dismeter",special_ultrasonic_offset_dismeter,static_cast<float>(0.15)); //特殊超声偏置距离

    p_nh->param("max_speed_v", max_speed_v, static_cast<double>(0.6));//最大速度
    p_nh->param("max_speed_w", max_speed_w, static_cast<double>(0.6));//最大角速度
    p_nh->param("speed_v_acc", speed_v_acc, static_cast<double>(0.025));//速度加速度
    p_nh->param("speed_v_dec", speed_v_dec, static_cast<double>(-0.12));//速度减速度
    p_nh->param("speed_v_dec_to_zero", speed_v_dec_zero, static_cast<double>(-0.12));
    p_nh->param("speed_w_acc", speed_w_acc, static_cast<double>(0.25));//角速度加速度
    p_nh->param("speed_w_dec", speed_w_dec, static_cast<double>(-0.25));//角速度减速度
    p_nh->param("full_speed",full_speed,static_cast<double>(3.0)); //电机满转速度
    p_nh->param("delta_counts_th",delta_counts_th,800); //码盘防抖动阈值
    p_nh->param("remote_speed_level", remote_speed_level_, 0);//遥控器控制速度
    p_nh->param("hardware_id", hardware_id, std::string("   "));//硬件设备名称
    p_nh->param("protector_num",protector_num,8);//防撞条使用数量
    p_nh->param("router_ip", router_ip, std::string("10.7.5.1"));//路由ip
    p_nh->param("laser_ip", laser_ip, std::string("10.7.5.100"));//激光ip
    p_nh->param("internet_url",internet_url,std::string("www.baidu.com"));//外网url用于测试外网状态
    p_nh->param("inplace_rotating_theta", inplace_rotating_theta, static_cast<double>(0.2));//初始化旋转速度
    p_nh->param("charger_low_voltage", charger_low_voltage_, static_cast<double>(24.5));//初始化
    p_nh->param("charger_full_voltage", charger_full_voltage_, static_cast<double>(27.5));//初始化旋转速度
    p_nh->param("new_hand_touch", new_hand_touch_, false);//新板子手触开关和防撞条共用一个接口
    p_nh->param("old_ultrasonic", old_ultrasonic_, false);//旧板子里面没有超声板状态
    p_nh->param("charger_delay_time",charger_delay_time_,30);//充电继电器打开延时时间

    // 前面防撞条配置
    if (!ReadConfigFromParams("front_protector", p_nh, &front_protector_list)) {
      GS_ERROR("[SERVICEROBOT] read front_protector_list failed");
    }
    // 后面防撞条配置
    if (!ReadConfigFromParams("rear_protector", p_nh, &rear_protector_list)) {
      GS_ERROR("[SERVICEROBOT] read rear_protector_list failed");
    }

    pthread_mutex_init(&speed_mutex, NULL);

    g_chassis_mcu->Init(host_name, std::to_string(port),f_dia, b_dia, axle,
                        timeWidth, counts, reduction_ratio, speed_ratio,
                        max_speed_v, max_speed_w, speed_v_acc, speed_v_dec,
                        speed_v_dec_zero, speed_w_acc, speed_w_dec,full_speed,delta_counts_th);

    GS_INFO("[wc_chassis] init param completed");
}

/* 设备的初始化*/
void InitDevice(void)
{

#ifdef VERIFY_REMOTE_KEY
  srand((unsigned int)time(NULL));
  unsigned int seed_key = rand();
  unsigned int check_key = GenerateJSHash(seed_key);
  unsigned int verify_key = g_chassis_mcu->checkRemoteVerifyKey(seed_key);
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
  g_chassis_mcu->setRemoteID((unsigned char)((remote_id & 0x0f) | ((remote_speed_level_ & 0x03) << 4) | ((battery_level_ & 0x03) << 6)));
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
     p_loop_rate =  new ros::Rate(10);

     InitService();
     InitParameter();
     InitDevice();
     InitSchedule();
     return true;
}
