#include"schedule.h"
#include"common_function.h"



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
            GS_ERROR("[chassis] unknown ping status:%d", ping_ret);
            return false;
        }
    } else {
        GS_ERROR("[chassis] ping program can not use");
        return false;
    }
}

/*
 * 检测激光＋路由器网关口+外网连接状态
*/
void checkConnectionHealthThread(void)
{
    while(1) {
        if (!ping(laser_ip.c_str()) || !ping(router_ip.c_str()) || !ping(internet_url.c_str())){
            sleep(3);
            if(!ping(laser_ip.c_str())){
                laser_connection_status = false;
            }
            if(!ping(router_ip.c_str())){
                router_connection_status = false;
            }
            if(!ping(internet_url.c_str())){
                internet_connection_status = false;
            }
        } else {
          laser_connection_status = true;
          router_connection_status = true;
          internet_connection_status = true;
        }
        sleep(30);
    }
}
