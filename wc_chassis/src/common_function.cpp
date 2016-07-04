/* common_function.cpp文件工程用到的所有的共有函数
*/
#include"common_function.h"
#include"init.h"

std::string get_key_value(int status, int status_bit)
{
  if (status & (0x01 << status_bit)) {
    return std::string("true");
  } else {
    return std::string("false");
  }
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
            ROS_ERROR("[chassis] unknown ping status:%d", ping_ret);
            return false;
        }
    } else {
        ROS_ERROR("[chassis] ping program can not use");
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
            if(!ping(laser_ip.c_str())){
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
  ROS_ERROR("[chassis] closed %d", system((std::string("kill -9 ") + str).c_str()));
  delete checkConnectionThread;
  delete g_chassis_mcu;
  delete p_odom_broadcaster;
  delete p_n;
  delete p_nh;
  delete p_device_nh;
  delete p_loop_rate;
  ros::shutdown();
}
