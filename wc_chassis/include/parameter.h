#ifndef __PARAMETER__
#define __PARAMETER__

extern  double ACC_LIM_TH ;
extern  double ultral_effective_range ;
extern  double g_odom_x   ;
extern  double g_odom_y   ;
extern  double g_odom_tha ;
extern  double g_odom_v ;
extern  double g_odom_w ;

enum Device_ID{
  Emergency_stop = 0,
  Mode,
  Battery,
  Mileage,
  Device_MAX
};

extern unsigned int g_dio_count ;
extern unsigned int g_ret_count ;
extern unsigned int g_pc_control;
extern double last_cmd_vel_time ;
extern double max_cmd_interval ;
extern int current_v_index;
extern int current_w_index;
extern float m_speed_v;
extern float m_speed_w ;
extern float g_speed_v[3];
extern float g_speed_w[3];
extern float g_spe;
extern float g_angle;
extern unsigned int loop_count;
extern unsigned int rotate_angle;
extern bool start_rotate_flag;
extern bool stop_rotate_flag;
extern bool is_rotate_finished;
extern unsigned int g_di_data_;
extern unsigned int g_do_data_;
extern unsigned int cur_emergency_status;
extern double battery_full_level;
extern double battery_empty_level;
extern unsigned int remote_ret_;
extern unsigned int remote_ret_cnt_;
extern unsigned char remote_cmd_;
extern unsigned short remote_index_;


#endif
