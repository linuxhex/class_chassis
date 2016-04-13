#include "init.h"

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
ros::Rate *loop_rate;

std::vector<int> g_ultrasonic;


ros::Publisher odom_pub;
ros::Publisher gyro_pub;
ros::Publisher remote_cmd_pub;
ros::Publisher device_pub;
ros::Publisher yaw_pub;
ros::Publisher ultrasonic0_pub;
ros::Publisher ultrasonic1_pub;
ros::Publisher ultrasonic2_pub;
ros::Publisher ultrasonic3_pub;
ros::Publisher ultrasonic4_pub;
ros::Publisher ultrasonic5_pub;


void DoNavigation(const geometry_msgs::Twist& Navigation_msg) {
  timeval tv;
  gettimeofday(&tv, NULL);
  last_cmd_vel_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;

  m_speed_v = Navigation_msg.linear.x;
  m_speed_w = Navigation_msg.angular.z;

//  ROS_INFO("Navigation.linear.x = %f, angular.z = %f", m_speed_v, m_speed_w);

  pthread_mutex_lock(&speed_mutex);
  double temp_v = Navigation_msg.linear.x;
  double temp_w = -1 * Navigation_msg.angular.z;
//  ROS_INFO("Navigation.linear.x = %f, angular.z = %f", temp_v, temp_w);
  int temp_v_index = (current_v_index + 1) % 3;
  int temp_w_index = (current_w_index + 1) % 3;
  g_speed_v[temp_v_index] = static_cast<float>(temp_v);
  g_speed_w[temp_w_index] = static_cast<float>(temp_w);
  current_v_index = temp_v_index;
  current_w_index = temp_w_index;
  pthread_mutex_unlock(&speed_mutex);
}

void RemoteRetCallback(const std_msgs::UInt32& ret) {
  remote_ret_ = ret.data;
  if (remote_ret_ & 0xff != 0) {
    remote_ret_cnt_ = 0;
  }
  ROS_INFO("[wc_chassis] get remote ret cmd = %d, state = %d", (remote_ret_ & 0xff), (remote_ret_ >> 8) & 0xff);
}

void GyroUpdateCallback(const std_msgs::UInt32& state) {
  if (state.data == 0) {
    g_chassis_mcu->gyro_state_ = 0;
  } else {
    g_chassis_mcu->gyro_state_ = 1;
  }
  ROS_INFO("[wc_chassis] set gyro state = %d", g_chassis_mcu->gyro_state_);
}

void publish_ultrasonic(ros::Publisher& publisher, const char* frame_id, int recv_int) {  // NOLINT
  sensor_msgs::Range range;
  range.header.seq = 0;
  range.header.stamp = ros::Time::now();
  range.header.frame_id = frame_id;

  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.field_of_view = M_PI / 90.0;
  range.min_range = 0.04;
  range.max_range = 1.0;

  float dis_meter = recv_int * 5.44 / 1000.0;
  if (dis_meter < range.min_range) {
    range.range = range.min_range;
  } else if (dis_meter > ultral_effective_range) {  // effective range
    range.range = range.max_range;
  } else {
    range.range = dis_meter;
  }
  publisher.publish(range);
}

void PublishUltrasonic() {
  publish_ultrasonic(ultrasonic0_pub, "ultrasonic0", g_ultrasonic[1]);
  publish_ultrasonic(ultrasonic1_pub, "ultrasonic1", g_ultrasonic[2]);
  publish_ultrasonic(ultrasonic2_pub, "ultrasonic2", g_ultrasonic[3]);
  publish_ultrasonic(ultrasonic3_pub, "ultrasonic3", g_ultrasonic[4]);
  publish_ultrasonic(ultrasonic4_pub, "ultrasonic4", g_ultrasonic[5]);
  publish_ultrasonic(ultrasonic5_pub, "ultrasonic5", g_ultrasonic[6]);
}

void PublishYaw(){
  std_msgs::Float32 msg;
  msg.data = g_odom_tha * 180.0 / M_PI;
  yaw_pub.publish(msg);
}

void PublishGyro() {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "world";

  tf::Quaternion temp;
  double roll, pitch, yaw;
  roll = g_chassis_mcu->roll_angle_ / 10.0 / 180.0 * M_PI;
  pitch = g_chassis_mcu->pitch_angle_ / 10.0 / 180.0 * M_PI;
  yaw = g_chassis_mcu->yaw_angle_ / 10.0 / 180.0 * M_PI;
  temp.setRPY(roll, pitch, yaw);
//#ifdef DEBUG_PRINT
//  std::cout << "roll = " << roll * 57.6 << "; pitch = " << pitch << ";yaw = " << std::endl;
//#endif
  imu_msg.orientation.x = temp.getX();
  imu_msg.orientation.y = temp.getY();
  imu_msg.orientation.z = temp.getZ();
  imu_msg.orientation.w = temp.getW();
  gyro_pub.publish(imu_msg);
}

void PublishOdom(tf::TransformBroadcaster* odom_broadcaster) {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();;

  odom.header.frame_id = "base_odom";
  // set the position
  odom.pose.pose.position.x = g_odom_x;
  odom.pose.pose.position.y = g_odom_y;
  odom.pose.pose.position.z = 0.0;
  int i;
  for (i = 0; i < 36; i++) odom.pose.covariance.elems[i] = 0.0;
  odom.pose.covariance.elems[0]  = 1.0;
  odom.pose.covariance.elems[7]  = 1.0;
  odom.pose.covariance.elems[14] = 1.0;
  odom.pose.covariance.elems[21] = 1.0;
  odom.pose.covariance.elems[28] = 1.0;
  odom.pose.covariance.elems[35] = 1.0;

  for (i = 0; i < 36; i++) odom.twist.covariance.elems[i] = 0.0;
  odom.twist.covariance.elems[0]  = 1.0;
  odom.twist.covariance.elems[7]  = 1.0;
  odom.twist.covariance.elems[14] = 1.0;
  odom.twist.covariance.elems[21] = 1.0;
  odom.twist.covariance.elems[28] = 1.0;
  odom.twist.covariance.elems[35] = 1.0;

  geometry_msgs::Quaternion odom_quat;
//  ROS_INFO("Odo Yaw = %lf", g_odom_tha * 57.3);
  odom_quat = tf::createQuaternionMsgFromYaw(g_odom_tha);
#ifdef DEBUG_PRINT
  std::cout << "odo based" << std::endl;
  std::cout << "x : " << Odomsg.x \
      << " y : " << Odomsg.y \
      << " angle : " << Odomsg.theta*180.0/3.1415926\
      << "Current Time-> " << now_s->tm_hour << ":" << now_s->tm_min << ":" << now_s->tm_sec << std::endl;
#endif

  odom.pose.pose.orientation = odom_quat;

  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = g_odom_v;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = g_odom_w;
  // publish the message
  odom_pub.publish(odom);

  tf::Quaternion q;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
  tf::Transform odom_meas(q, tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0));
  tf::StampedTransform odom_transform(odom_meas, ros::Time::now(), "base_odom", "base_link");
  odom_broadcaster->sendTransform(odom_transform);
}

void PublisheRemoteCmd(unsigned char cmd, unsigned short index) {
  std_msgs::UInt32 remote_cmd;
  if (cmd > 0 && cmd < 12) {
    ROS_INFO("[wc_chassis] get remote cmd = %d, index = %d", cmd, index);
  }
  remote_cmd.data = (index << 8) | cmd;
  remote_cmd_pub.publish(remote_cmd);
}

void publish_device_status() {
  diagnostic_msgs::DiagnosticStatus device_status;
  diagnostic_msgs::KeyValue device_value;
  device_status.level = diagnostic_msgs::DiagnosticStatus::OK;
  device_status.name = std::string("device_status");
  device_status.message = std::string("status_msgs");
  device_status.hardware_id = std::string("status_msgs");

  device_value.key = std::string("emergency_stop");
  device_value.value = cur_emergency_status == 0 ? std::string("true") : std::string("false");
  device_status.values.push_back(device_value);

  unsigned int battery_ADC = (g_ultrasonic[20] << 2) | (g_ultrasonic[21] & 0x03);
  double battery_value = (0.2393 * battery_ADC - 125.04) * 100;
  int battery_capacity;
  if(battery_value <= battery_empty_level) {
    battery_capacity = 0;
  } else if(battery_value >= battery_full_level) {
    battery_capacity = 100;
  } else {
    battery_capacity = (battery_value - battery_empty_level) / (battery_full_level - battery_empty_level) * 100;
  }
/*
  } else if(battery_value >= 2350) {
    battery_capacity = (battery_value - 2350) * 40 / (2750 - 2350) + 60;
  } else {
    battery_capacity = (battery_value - 2000) * 60 / (3500 - 3100);
  }
  battery_capacity = battery_capacity > 100 ? 100 : battery_capacity;
  battery_capacity = battery_capacity < 0 ? 0 : battery_capacity;
*/
  std::cout << "battery_ADC " << battery_ADC << "; battery_balue " << battery_value << "; battery_capacity " << battery_capacity << std::endl;
  device_value.key = std::string("battery");
  device_value.value = std::to_string(battery_capacity);
  device_status.values.push_back(device_value);

  device_value.key = std::string("mileage");
  unsigned int mileage = (unsigned int)((g_chassis_mcu->mileage_left_ + g_chassis_mcu->mileage_left_) / 2);
  device_value.value = std::to_string(mileage);
  device_status.values.push_back(device_value);

  device_pub.publish(device_status);
}

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

bool StartRotate(autoscrubber_services::StartRotate::Request& req, autoscrubber_services::StartRotate::Response& res) {
  rotate_angle = req.rotateAngle.data;
  start_rotate_flag = true;
  is_rotate_finished = false;
  g_chassis_mcu->acc_odom_theta_ = 0.0;
  return true;
}

bool StopRotate(autoscrubber_services::StopRotate::Request& req, autoscrubber_services::StopRotate::Response& res) {
  start_rotate_flag = false;
  return true;
}

bool CheckRotate(autoscrubber_services::CheckRotate::Request& req, autoscrubber_services::CheckRotate::Response& res) {
  res.isFinished.data = is_rotate_finished;
  return true;
}

/***
 * 初始化所有的Service和订阅服务
 */
void InitService(ros::NodeHandle n,
                 ros::NodeHandle nh,
                 ros::NodeHandle device_nh){
    device_nh.advertiseService("start_rotate", &StartRotate);
    device_nh.advertiseService("stop_rotate", &StopRotate);
    device_nh.advertiseService("check_rotate", &CheckRotate);
    n.subscribe("cmd_vel", 10, DoNavigation);
    device_nh.subscribe("/remote_ret", 10, RemoteRetCallback);
    n.subscribe("/gyro_update_state", 10, GyroUpdateCallback);
}

/***
 * 初始化所有的Publish服务
 */
void InitPublisher(ros::NodeHandle n,
                   ros::NodeHandle nh,
                   ros::NodeHandle device_nh){
    // yaw_pub = n.advertise<std_msgs::Float32>("yaw", 10);
    odom_pub  = n.advertise<nav_msgs::Odometry>("odom", 50);
    gyro_pub  = device_nh.advertise<sensor_msgs::Imu>("gyro", 50);
    remote_cmd_pub  = device_nh.advertise<std_msgs::UInt32>("remote_cmd", 50);
    device_pub = device_nh.advertise<diagnostic_msgs::DiagnosticStatus>("device_status", 50);
    ultrasonic0_pub = n.advertise<sensor_msgs::Range>("ultrasonic0", 50);
    ultrasonic1_pub = n.advertise<sensor_msgs::Range>("ultrasonic1", 50);
    ultrasonic2_pub = n.advertise<sensor_msgs::Range>("ultrasonic2", 50);
    ultrasonic3_pub = n.advertise<sensor_msgs::Range>("ultrasonic3", 50);
    ultrasonic4_pub = n.advertise<sensor_msgs::Range>("ultrasonic4", 50);
    ultrasonic5_pub = n.advertise<sensor_msgs::Range>("ultrasonic5", 50);
}

/***
 *  用到的所有的参数的初始化
 */
void InitParameter(ros::NodeHandle n,
                   ros::NodeHandle nh,
                   ros::NodeHandle device_nh){

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
    nh.param("odom", str_odom, str_odom);
    nh.param("WC_Auto_topic", str_auto_topic, std::string("WC_AUTO"));
    nh.param("max_cmd_interval", max_cmd_interval, 1.0);
    nh.param("F_DIA", f_dia, static_cast<double>(0.125));	// diameter of front wheel
    nh.param("B_DIA", b_dia, static_cast<double>(0.125));
    nh.param("AXLE", axle, static_cast<double>(0.383));		// length bettween two wheels
    nh.param("COUNTS", counts, 12);
    nh.param("REDUCTION_RATIO", reduction_ratio, 30);
    nh.param("SPEED_RATIO", speed_ratio, static_cast<double>(1.0));
    nh.param("TimeWidth", timeWidth, static_cast<double>(0.1));
    nh.param("ultral_effective_range", ultral_effective_range, static_cast<double>(0.4));
    nh.param("host_name", host_name, std::string("192.168.1.199"));
    nh.param("port", port, 5000);
    nh.param("acc_lim_th", ACC_LIM_TH, 3.0 / 2.0 * M_PI);
    nh.param("battery_full_level", battery_full_level, static_cast<double>(27.5));
    nh.param("battery_empty_level", battery_empty_level, static_cast<double>(20.0));
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
     ros::NodeHandle n;
     ros::NodeHandle nh("~");
     ros::NodeHandle device_nh("device");

     InitService(n,nh,device_nh);
     InitPublisher(n,nh,device_nh);
     InitParameter(n,nh,device_nh);
     loop_rate =  new ros::Rate(10);
     return true;
}
