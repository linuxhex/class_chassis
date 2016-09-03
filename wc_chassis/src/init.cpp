/* init.cpp文件包含了所有wc_chassis工程的初始化
*/

#include "init.h"
#include "subscribe.h"
#include "parameter.h"
#include "common_function.h"
#include "schedule.h"

/***ros 相关****/
tf::TransformBroadcaster *p_odom_broadcaster = NULL;
ros::Rate *p_loop_rate = NULL;
ros::NodeHandle *p_n = NULL;
ros::NodeHandle *p_nh = NULL;
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

    p_chassis_mcu = new WC_chassis_mcu();
    p_odom_broadcaster = new tf::TransformBroadcaster();
    ultrasonic = new std::string();
    special_ultrasonic = new std::string();
    double speed_ratio = 1.0;

//    p_nh->param("max_cmd_interval", max_cmd_interval, 1.0);
//    p_nh->param("TimeWidth", timeWidth, static_cast<double>(0.1));
  //  p_nh->param("host_name", host_name, std::string("10.7.5.199"));
  //  p_nh->param("port", port, 5000);
    //    p_nh->param("router_ip", router_ip, std::string("10.7.5.1"));//路由ip
    //    p_nh->param("laser_ip", laser_ip, std::sscp tring("10.7.5.100"));//激光ip
    //    p_nh->param("internet_url",internet_url,std::string("www.baidu.com"));//外网url用于测试外网状态

//    p_nh->param("F_DIA", f_dia, static_cast<double>(0.125));	// diameter of front wheel
//    p_nh->param("B_DIA", b_dia, static_cast<double>(0.125));
//    p_nh->param("AXLE", axle, static_cast<double>(0.383));		// length bettween two wheels
//    p_nh->param("COUNTS", counts, 12);//霍尔数
//    p_nh->param("REDUCTION_RATIO", reduction_ratio, static_cast<double>(30.0));//减速比
//    p_nh->param("SPEED_RATIO", speed_ratio, static_cast<double>(1.0));
//    p_nh->param("battery_full_level", battery_full_level, static_cast<double>(27.5));
//    p_nh->param("battery_empty_level", battery_empty_level, static_cast<double>(20.0));

//    p_nh->param("ultrasonic",*ultrasonic,std::string(" "));//配置的超声
//    p_nh->param("ultrasonic_min_range",ultrasonic_min_range,static_cast<float>(0.04));//超声最小距离
//    p_nh->param("ultrasonic_max_range",ultrasonic_max_range,static_cast<float>(1.0));//超声最大距离
//    p_nh->param("ultral_effective_range", ultral_effective_range, static_cast<double>(0.4));//超声有效检测距离
//    p_nh->param("special_ultrasonic",*special_ultrasonic,std::string(" "));//特殊配置的超声
//    p_nh->param("special_ultrasonic_offset_dismeter",special_ultrasonic_offset_dismeter,static_cast<float>(0.15)); //特殊超声偏置距离

//    p_nh->param("max_speed_v", max_speed_v, static_cast<double>(0.6));//最大速度
//    p_nh->param("max_speed_w", max_speed_w, static_cast<double>(0.6));//最大角速度
//    p_nh->param("speed_v_acc", speed_v_acc, static_cast<double>(0.025));//速度加速度
//    p_nh->param("speed_v_dec", speed_v_dec, static_cast<double>(-0.12));//速度减速度
//    p_nh->param("speed_v_dec_to_zero", speed_v_dec_zero, static_cast<double>(-0.12));
//    p_nh->param("speed_w_acc", speed_w_acc, static_cast<double>(0.25));//角速度加速度
//    p_nh->param("speed_w_dec", speed_w_dec, static_cast<double>(-0.25));//角速度减速度
//    p_nh->param("full_speed",full_speed,static_cast<double>(3.0)); //电机满转速度
//    p_nh->param("delta_counts_th",delta_counts_th,800); //码盘防抖动阈值
//    p_nh->param("remote_speed_level", remote_speed_level_, 0);//遥控器控制速度
//    p_nh->param("hardware_id", hardware_id, std::string("   "));//硬件设备名称
//    p_nh->param("protector_num",protector_num,8);//防撞条使用数量
//    p_nh->param("router_ip", router_ip, std::string("10.7.5.1"));//路由ip
//    p_nh->param("laser_ip", laser_ip, std::sscp tring("10.7.5.100"));//激光ip
//    p_nh->param("internet_url",internet_url,std::string("www.baidu.com"));//外网url用于测试外网状态
//    p_nh->param("inplace_rotating_theta", inplace_rotating_theta, static_cast<double>(0.2));//初始化旋转速度
//    p_nh->param("charger_low_voltage", charger_low_voltage_, static_cast<double>(24.5));//初始化
//    p_nh->param("charger_full_voltage", charger_full_voltage_, static_cast<double>(27.5));//初始化旋转速度
//    p_nh->param("new_hand_touch", new_hand_touch_, false);//新板子手触开关和防撞条共用一个接口
//    p_nh->param("old_ultrasonic", old_ultrasonic_, false);//旧板子里面没有超声板状态
//    p_nh->param("charger_delay_time",charger_delay_time_,30);//充电继电器打开延时时间

    //read device param
    ros::NodeHandle chassis_param_nh("~/chassis_param");
    std::string device_params;
    chassis_param_nh.param("device", device_params, std::string(""));
    std::cout<<"device"<<" "<<device_params<<std::endl;

    std::stringstream device_ss(device_params);
    std::string device_param;
    while (device_ss >> device_param) {
      if (device_param == "charger") {
          ros::NodeHandle charger_nh("~/chassis_param/charger");
          charger_nh.param("low_voltage", charger_low_voltage_, static_cast<double>(24.5));//初始化
          charger_nh.param("full_voltage", charger_full_voltage_, static_cast<double>(27.5));//初始化旋转速度
          charger_nh.param("delay_time",charger_delay_time_,30);//充电继电器打开延时时间
      } else if (device_param == "protector") {
          ros::NodeHandle protector_nh("~/chassis_param/protector");
          protector_nh.param("protector_num",protector_num,8);//防撞条使用数量
      } else if (device_param == "hand_touch") {
          ros::NodeHandle hand_touch_nh("~/chassis_param/hand_touch");
          hand_touch_nh.param("new_hand_touch", new_hand_touch_, false);//新板子手触开关和防撞条共用一个接口

      } else if (device_param == "ultrasonic") {
          ros::NodeHandle ultrasonic_nh("~/chassis_param/ultrasonic");
          ultrasonic_nh.param("name",*ultrasonic,std::string(" "));//配置的超声
          ultrasonic_nh.param("min_range",ultrasonic_min_range,static_cast<float>(0.04));//超声最小距离
          ultrasonic_nh.param("max_range",ultrasonic_max_range,static_cast<float>(1.0));//超声最大距离
          ultrasonic_nh.param("effective_range", ultral_effective_range, static_cast<double>(0.4));//超声有效检测距离
          ultrasonic_nh.param("specialer",*special_ultrasonic,std::string(" "));//特殊配置的超声
          ultrasonic_nh.param("specialer_offset_dismeter",special_ultrasonic_offset_dismeter,static_cast<float>(0.15)); //特殊超声偏置距离

      } else if (device_param == "battery") {
          ros::NodeHandle battery_nh("~/chassis_param/battery");
          battery_nh.param("full_level", battery_full_level, static_cast<double>(27.5));
          battery_nh.param("empty_level", battery_empty_level, static_cast<double>(20.0));

      } else if (device_param == "machine") {
          p_machine = new Machine();

      } else if (device_param == "network") {
          p_network = new Network();

          network_nh.param("laser_ip", laser_ip, std::string("10.7.5.100"));//激光ip
          network_nh.param("internet_url",internet_url,std::string(""));//外网url用于测试外网状态

      } else if (device_param == "checker_id") {
          ros::NodeHandle checker_id_nh("~/chassis_param/checker_id");
          checker_id_nh.param("hardware_id", hardware_id, std::string("   "));//硬件设备名称

      } else if (device_param == "footprint") {
          ros::NodeHandle footprint_nh("~/chassis_param/footprint");

      } else if (device_param == "laser") {
          ros::NodeHandle laser_nh("~/chassis_param/laser");

      }
    }

    //read strategy param
    std::string strategy_params;
    chassis_param_nh.param("strategy", strategy_params, std::string(""));
    std::stringstream strategy_ss(strategy_params);
    std::string strategy_param;
    while (strategy_ss >> strategy_param) {
      if (strategy_param == "speed_v") {
          ros::NodeHandle speed_v_nh("~/chassis_param/speed_v");
          speed_v_nh.param("max", max_speed_v, static_cast<double>(0.6));//最大速度
          speed_v_nh.param("acc", speed_v_acc, static_cast<double>(0.025));//速度加速度
          speed_v_nh.param("dec", speed_v_dec, static_cast<double>(-0.12));//速度减速度
          speed_v_nh.param("dec_to_zero", speed_v_dec_zero, static_cast<double>(-0.12));
          speed_v_nh.param("full",full_speed,static_cast<double>(3.0)); //电机满转速度
          speed_v_nh.param("remote_level", remote_speed_level_, 0);//遥控器控制速度

      } else if (strategy_param == "speed_w") {
          ros::NodeHandle speed_w_nh("~/chassis_param/speed_w");
          speed_w_nh.param("max", max_speed_w, static_cast<double>(0.6));//最大角速度
          speed_w_nh.param("speed_w_acc", speed_w_acc, static_cast<double>(0.25));//角速度加速度
          speed_w_nh.param("speed_w_dec", speed_w_dec, static_cast<double>(-0.25));//角速度减速度
          speed_w_nh.param("inplace_rotating_theta", inplace_rotating_theta, static_cast<double>(0.2));//初始化旋转速度


      }
    }

//    // 前面防撞条配置
//    if (!ReadConfigFromParams("front_protector", p_nh, &front_protector_list)) {
//      GS_ERROR("[SERVICEROBOT] read front_protector_list failed");
//    }
//    // 后面防撞条配置
//    if (!ReadConfigFromParams("rear_protector", p_nh, &rear_protector_list)) {
//      GS_ERROR("[SERVICEROBOT] read rear_protector_list failed");
//    }

    pthread_mutex_init(&speed_mutex, NULL);

    p_chassis_mcu->Init(
                        speed_ratio,
                        max_speed_v, max_speed_w, speed_v_acc, speed_v_dec,
                        speed_v_dec_zero, speed_w_acc, speed_w_dec,full_speed);

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
  p_chassis_mcu->setRemoteID((unsigned char)((remote_id & 0x0f) | ((remote_speed_level_ & 0x03) << 4) | ((battery_level_ & 0x03) << 6)));
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
