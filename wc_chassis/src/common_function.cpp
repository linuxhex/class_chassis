/* common_function.cpp文件工程用到的所有的共有函数
*/
#include"common_function.h"
#include"init.h"

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
            ROS_ERROR("[checker] unknown ping status:%d", ping_ret);
            return false;
        }
    } else {
        ROS_ERROR("[checker] ping program can not use");
        return false;
    }
}

/*
 * 检测激光＋路由器网关口连接状态
*/
void checkConnectionHealthThread(void) {

    while(1){
        if(!ping(laser_ip.c_str()) || !ping(router_ip.c_str())){
            sleep(1);
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
        sleep(15);
    }
}
