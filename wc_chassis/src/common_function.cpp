/* common_function.cpp文件工程用到的所有的共有函数
*/
#include"common_function.h"
#include"init.h"
#include"parameter.h"

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

/*
 * ping ip
*/
bool ping(const char* ip) {
    int status, ping_ret;
    status = system((std::string("ping -w 1 ") + std::string(ip)).c_str());
    if (-1 != status) {
        ping_ret = WEXITSTATUS(status);
        if (0 == ping_ret) {
            return true;
        } else if (1 == ping_ret) {
            return false;
        } else {
            GAUSSIAN_ERROR("[chassis] unknown ping status:%d", ping_ret);
            return false;
        }
    } else {
        GAUSSIAN_ERROR("[chassis] ping program can not use");
        return false;
    }
}

/*
 * 检测激光＋路由器网关口连接状态
*/
void checkConnectionHealthThread(void) {

    while(1){
        if(!ping(laser_ip.c_str()) || !ping(router_ip.c_str())){
            sleep(3);
            if(!ping(laser_ip.c_str())){
                laser_connection_status = std::string("false");
            }
            if(!ping(router_ip.c_str())){
                router_connection_status = std::string("false");
            }
        }else{
          laser_connection_status = std::string("true");
          router_connection_status = std::string("true");
        }
        sleep(20);
    }
}

void freeResource(void){

  std::stringstream ss;
  std::string str;
  ss << checkConnectionThread->get_id();
  ss >> str;
  GAUSSIAN_ERROR("[chassis] closed %d", system((std::string("kill -9 ") + str).c_str()));
  delete checkConnectionThread;
  delete g_chassis_mcu;
  delete p_odom_broadcaster;
  delete p_n;
  delete p_nh;
  delete p_device_nh;
  delete p_loop_rate;
  delete p_io;
  delete p_update_device_timer;
  ros::shutdown();
}
