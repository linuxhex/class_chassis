/* publish.cpp 所有的topic的发布
 */

#include "publish.h"
#include "init.h"
#include "parameter.h"

/*
 * 发送超声
 */
void publish_ultrasonic(ros::Publisher& publisher, const char* frame_id, int recv_int,unsigned int ultrasonic_offset, double& ultra_range) {  // NOLINT
  sensor_msgs::Range range;
  range.header.seq = 0;
  range.header.stamp = ros::Time::now();
  range.header.frame_id = frame_id;

  range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range.field_of_view = M_PI / 90.0;
  range.min_range = ultrasonic_min_range;
  range.max_range = ultrasonic_max_range;

  float dis_meter = recv_int * 5.44 / 1000.0;
  ultra_range = dis_meter;

  if(special_ultrasonic_id[ultrasonic_offset] == ultrasonic_offset){ //特殊位置超声处理
    dis_meter = dis_meter - special_ultrasonic_offset_dismeter;
  }
  if (dis_meter < range.min_range) {
    range.range = range.min_range;
  } else if (dis_meter > ultral_effective_range) {  // effective range
    range.range = range.max_range;
  } else {
    range.range = dis_meter;
  }

  if(((ultrasonic_bits & (0x01<<ultrasonic_offset)) != 0x00) || !ultrasonic_board_connection){ //应用层屏蔽超声的作用 或者超声转接板断开连接
    range.range = ultrasonic_max_range;
  }

  publisher.publish(range);
}


void PublishUltrasonic(ros::Publisher ultrasonic_pub[]) {
  double raw_range;
  for(int i=0;i<15;i++){
    if(ultrasonic_pub[i] != 0){  //==0表示不是文件里配置的超声
      publish_ultrasonic(ultrasonic_pub[i], ultrasonic_str[i].c_str(), g_ultrasonic[1+i], i, raw_range);
      GS_INFO("[wc chassis] get ultra[%d] raw range = %lf", i, raw_range);
    }
  }
}

/*
 * 发送遥控器命令
 */
void PublishRemoteCmd(ros::Publisher &remote_cmd_pub,unsigned char cmd, unsigned short index) {
  std_msgs::UInt32 remote_cmd;
  if (cmd > 0 && cmd < 12) {
    GS_INFO("[wc_chassis] get remote cmd = %d, index = %d", cmd, index);
  }
  remote_cmd.data = (index << 8) | cmd;
  remote_cmd_pub.publish(remote_cmd);
}

/*
 * 发送设备状态
 */
void publishDeviceStatus(ros::Publisher &device_pub) {

    diagnostic_msgs::DiagnosticStatus device_status;
    diagnostic_msgs::KeyValue device_value;
    device_status.level = diagnostic_msgs::DiagnosticStatus::OK;
    device_status.name = std::string("device_status");
    device_status.message = std::string("status_msgs");
    device_status.hardware_id = std::string("status_msgs");

    device_value.key = std::string("emergency_stop");
    device_value.value = cur_emergency_status == 0 ? std::string("true") : std::string("false");
    device_status.values.push_back(device_value);

    unsigned int battery_ADC = (g_ultrasonic[20] << 8) | (g_ultrasonic[21] & 0xff);
//    double battery_value = 0.2298 * (battery_ADC - 516);
//    double battery_value = 0.2352 * (battery_ADC - 507);
    double battery_value = 0.2460 * (battery_ADC - 516);
    battery_value = battery_value < 0.0 ? 0.0 : battery_value;
    battery_value = battery_value > 35.0 ? 0.0 : battery_value;
    int current_battery_capacity;
    current_battery_capacity = (battery_value - battery_empty_level) / (battery_full_level - battery_empty_level) * 100;
    if(current_battery_capacity < 0) current_battery_capacity = 0;
    if(current_battery_capacity > 100) current_battery_capacity = 100;
    // do battery voltage filter
    ++battery_count;
    if(battery_count == 0) {
      battery_value_ = battery_value;
      display_battery_capacity = current_battery_capacity;
    } else if(battery_count > 0) {
      sum_battery_capacity += current_battery_capacity;
      sum_battery_value_ += battery_value;
      if(battery_count == 300) {
        display_battery_capacity = sum_battery_capacity / 300;
        battery_value_ = sum_battery_value_ / 300;
        battery_count = 0;
        sum_battery_capacity = 0;
        sum_battery_value_ = 0.0;
      }
    }

    if (display_battery_capacity < 10) {
      battery_level_ = 0;
    } else if (display_battery_capacity < 40) {
      battery_level_ = 1;
    } else if (display_battery_capacity < 75) {
      battery_level_ = 2;
    } else {
      battery_level_ = 3;
    }

    GS_INFO("[wc_chassis] battery_ADC: %d; battery_value: %lf;display_battery_capacity: %d; battery_level_: %d", battery_ADC, battery_value, display_battery_capacity, battery_level_);
//    std::cout << "battery_ADC " << battery_ADC << "; battery_value " << battery_value << "; current_battery_capacity " << current_battery_capacity << "; display_battery_capacity " << display_battery_capacity << " battery_level_" << battery_level_ << std::endl;
    device_value.key = std::string("battery");
    device_value.value = std::to_string(display_battery_capacity);
    device_status.values.push_back(device_value);

    device_value.key = std::string("battery_voltage");
    device_value.value = std::to_string(battery_value_);
    device_status.values.push_back(device_value);

    device_value.key = std::string("charger_status");
    if (charger_status_ == STA_CHARGER_ON) {
       device_value.value = std::string("true");
    } else {
       device_value.value = std::string("false");
    }
    device_status.values.push_back(device_value);

    device_value.key = std::string("charger_voltage");
    device_value.value = std::to_string(charger_voltage_);
    device_status.values.push_back(device_value);

    device_value.key = std::string("mileage");
    double mileage = (g_chassis_mcu->mileage_right_ + g_chassis_mcu->mileage_left_) / 2;
    device_value.value = std::to_string(mileage);
    device_status.values.push_back(device_value);

    device_pub.publish(device_status);
}
/*
 * 发送防撞条数据给导航
 */
void publish_protector_value(ros::Publisher &protector_status_pub){
    std_msgs::UInt32 protect_data;
    if(new_hand_touch_){
        protector_value = NONE_HIT;
    }
    protect_data.data = protector_value;
    protector_status_pub.publish(protect_data);
}
/*
 * 发送防撞条状态
 */
void publish_protector_status(ros::Publisher &protector_pub) {

  if(protector_num <= 0){
      return;
  }
  std::bitset<32> status;
  std::string str;
  diagnostic_msgs::KeyValue value;
  status = g_ultrasonic[0] | protector_bits;
  str = status.to_string();
  value.key = std::string("protector_data"); // 0:on 1:off
  value.value = str.substr((32-protector_num), protector_num);

  for(unsigned int i = 0; i < value.value.length()/2; i++ ){
      char c = value.value[i];
      value.value[i] = value.value[value.value.length() -1 -i];
      value.value[value.value.length() -1 -i] = c;
  }
  protector_pub.publish(value);
}

/*
 * 发送里程值
 */
void PublishOdom(tf::TransformBroadcaster* odom_broadcaster,ros::Publisher &odom_pub ) {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();;

  odom.header.frame_id = "base_odom";
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
  odom_pub.publish(odom);

  ros::Time odom_timestamped = ros::Time::now() + ros::Duration(0.1);
  tf::Quaternion q;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
  tf::Transform odom_meas(q, tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0));
  tf::StampedTransform odom_transform(odom_meas, ros::Time::now(), "base_odom", "base_link");
  odom_broadcaster->sendTransform(odom_transform);
}

void PublishDIO(ros::Publisher &dio_pub) {
  std_msgs::UInt32 dio_data;
  dio_data.data = g_di_data_;
  dio_pub.publish(dio_data);
}

void PublishYaw(ros::Publisher &yaw_pub){
  std_msgs::Float32 msg;
  msg.data = g_odom_tha * 180.0 / M_PI;
  yaw_pub.publish(msg);
}

