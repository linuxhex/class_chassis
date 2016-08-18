/* common_function.cpp文件工程用到的所有的共有函数
*/
#include"common_function.h"
#include"init.h"
#include"parameter.h"


/*
 * 防撞条数据处理
*/
void protectorManage(void)
{
    if(protector_num <= 0){
        return;
    }
    // check protector hit
    unsigned int protector_status = g_ultrasonic[0] | protector_bits;
    unsigned int temp_hit = NONE_HIT;
    for (unsigned int i = 0; i < front_protector_list.size(); ++i) {
      if (!(protector_status & (1 << front_protector_list.at(i)))) {
        temp_hit |= FRONT_HIT;
        GAUSSIAN_ERROR("[WC_CHASSIS] front protector bit[%d] hit!!!", front_protector_list.at(i));
        break;
      }
    }
    for (unsigned int i = 0; i < rear_protector_list.size(); ++i) {
     if (!(protector_status & (1 << rear_protector_list.at(i)))) {
        temp_hit |= REAR_HIT;
        GAUSSIAN_ERROR("[WC_CHASSIS] rear protector bit[%d] hit!!!", rear_protector_list.at(i));
        break;
      }
    }
    if (temp_hit != NONE_HIT) {
       timeval tv;
       protector_value |= temp_hit;
       gettimeofday(&tv, NULL);
       protector_hit_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
     }

    timeval tv;
    gettimeofday(&tv, NULL);
    double time_now = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
    //超过１秒，自动清除给导航的状态
    if((protector_value != NONE_HIT) && (time_now - protector_hit_time > max_cmd_interval)){
      protector_value = NONE_HIT;
    }

    protector_hit = temp_hit;

}
/*
 * 充电电压数据处理
*/
void chargeValueManage(void)
{
    unsigned int charge_ADC = (g_ultrasonic[22] << 8) | (g_ultrasonic[23] & 0xff);
    //  double current_charge_voltage = 0.2298 * (charge_ADC - 516);
    //  double current_charge_voltage = 0.2352 * (charge_ADC - 507);
    double current_charge_voltage = 0.2398 * (charge_ADC - 512);
    current_charge_voltage = current_charge_voltage < 10.0 ? 0.0 : current_charge_voltage;
    current_charge_voltage = current_charge_voltage > 50.0 ? 0.0 : current_charge_voltage;
    if (charger_cmd_ == CMD_CHARGER_MONITOR && charger_voltage_ < charger_low_voltage_ && current_charge_voltage > charger_low_voltage_) {
      timeval tv;
      gettimeofday(&tv, NULL);
      charge_on_time_ = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
     }
    charger_voltage_ = current_charge_voltage;
    GAUSSIAN_INFO("[wc_chassis] adc_charge = %d, charger_voltage_: %lf", charge_ADC, charger_voltage_);

    relay_status_ = g_chassis_mcu->setChargeCmd(0);
    if ((relay_status_ & 0x03) == STA_CHARGER_ON) {
      charger_status_ = STA_CHARGER_ON;
    } else if (charger_voltage_ >= charger_low_voltage_) {
      charger_status_ = STA_CHARGER_TOUCHED;
    } else {
      charger_status_ = STA_CHARGER_OFF;
    }
    GAUSSIAN_INFO("[wc_chassis] get relay status = 0x%.2x, charger_status_ = %d", relay_status_, charger_status_);

}

/*
 * 获取防撞条＋超声这些设备的状态数据＋充电电压的处理
*/
void updateDeviceStatus(void)
{
  /*超声can连接状处理*/
  if (!old_ultrasonic_) {
     std::istringstream(get_key_value(g_ultrasonic[19], Ultrasonic_board)) >> std::boolalpha >> ultrasonic_board_connection;
  }
  chargeValueManage();
  protectorManage();
}


void ReadConfigFromXMLRPC(XmlRpc::XmlRpcValue& config_xmlrpc, const std::string& full_param_name, std::vector<unsigned int>* config_list) {
  unsigned int config;
  for (int i = 0; i < config_xmlrpc.size(); ++i) {
    // Make sure each element of the list is an array of size 2. (x and y coordinates)
    XmlRpc::XmlRpcValue value = config_xmlrpc[ i ];
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
      ROS_FATAL("The config (parameter %s) must be specified as list of parameter server eg: [1, 2, ..., n], but this spec is not of that form.", full_param_name.c_str());
      throw std::runtime_error( "The config must be specified as list of parameter server eg: [1, 2, ..., n],, but this spec is not of that form");
    }
    config = static_cast<int>(value);
    GAUSSIAN_INFO("[SERVICEROBOT] get config[%d] px = %d", i, config);
    config_list->push_back(config);
  }
}

bool ReadConfigFromParams(std::string param_name, ros::NodeHandle* nh, std::vector<unsigned int>* config_list) {
  std::string full_param_name;

  if (nh->searchParam(param_name, full_param_name)) {
    XmlRpc::XmlRpcValue config_xmlrpc;
    nh->getParam(full_param_name, config_xmlrpc);
    GAUSSIAN_INFO("[CHASSIS] %s: config_list size = %zu",param_name.c_str(), config_xmlrpc.size());
    if (config_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray && config_xmlrpc.size() > 0) {
      ReadConfigFromXMLRPC(config_xmlrpc, full_param_name, config_list);
      for (unsigned int i = 0; i < config_list->size(); ++i) {
        GAUSSIAN_INFO("[CHASSIS] %s: config_list[%d/%zu] = %d",param_name.c_str() ,i, config_list->size(), config_list->at(i));
      }
      return true;
    } else {
      GAUSSIAN_ERROR("[CHASSIS] %s: config_list param's type is not Array!", param_name.c_str());
    }
  }
  return false;
}

std::string get_key_value(int status, int status_bit)
{
  if (status & (0x01 << status_bit)) {
    return std::string("true");
  } else {
    return std::string("false");
  }
}

/* 　设置chassis调度优先级，cpu*/
void SetSchedPriority(void)
{
    #ifdef SETTING_PRIORITY
        struct sched_param param;
        param.sched_priority = 99;
        if (0 != sched_setscheduler(getpid(), SCHED_RR, &param)) {
          std::cout << "set priority failed" << std::endl;
        } else {
          std::cout << "set priority succeed" << std::endl;
        }
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(0, &mask);
        if (sched_setaffinity(0, sizeof(mask), &mask) < 0) {
          std::cout << "set affinity failed" << std::endl;
        } else {
          std::cout << "set affinity succeed" << std::endl;
        }
    #endif
}

#ifdef VERIFY_REMOTE_KEY
unsigned int GenerateJSHash(unsigned int seed) {
  unsigned char ch[] = "NTM4N2I2YmFiOWIwNzgzYmViYWFjYjc2";
  unsigned int hash = seed;
  for(int i = 0; i < 32; i++) {
    hash ^= ((hash << 5) + ch[i] + (hash >> 2));
  }
  for(int i = 0; i < 32; i++) {
    hash ^= (((hash - ch[i]) >> 2) + (hash << 3));
  }
  return hash;
}
#endif


void freeResource(void){

  std::stringstream ss;
  std::string str;
  ss << p_checkConnectionThread->get_id();
  ss >> str;
  GAUSSIAN_ERROR("[chassis] closed %d", system((std::string("kill -9 ") + str).c_str()));

  delete p_checkConnectionThread;
  delete g_chassis_mcu;
  delete p_odom_broadcaster;
  delete p_n;
  delete p_nh;
  delete p_device_nh;
  delete p_loop_rate;

  ros::shutdown();
}
