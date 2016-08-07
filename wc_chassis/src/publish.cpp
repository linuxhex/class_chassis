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
      GAUSSIAN_INFO("[wc chassis] get ultra[%d] raw range = %lf", i, raw_range);
    }
  }
}
/*
 * 发送陀螺仪数据
 */
void PublishGyro(ros::Publisher &gyro_pub) {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "world";

  tf::Quaternion temp;
  double roll, pitch, yaw;
  roll = g_chassis_mcu->roll_angle_ / 10.0 / 180.0 * M_PI;
  pitch = g_chassis_mcu->pitch_angle_ / 10.0 / 180.0 * M_PI;
  yaw = g_chassis_mcu->yaw_angle_ / 10.0 / 180.0 * M_PI;
  temp.setRPY(roll, pitch, yaw);

  imu_msg.orientation.x = temp.getX();
  imu_msg.orientation.y = temp.getY();
  imu_msg.orientation.z = temp.getZ();
  imu_msg.orientation.w = temp.getW();
  gyro_pub.publish(imu_msg);
}
/*
 * 发送遥控器命令
 */
void PublishRemoteCmd(ros::Publisher &remote_cmd_pub,unsigned char cmd, unsigned short index) {
  std_msgs::UInt32 remote_cmd;
  if (cmd > 0 && cmd < 12) {
    GAUSSIAN_INFO("[wc_chassis] get remote cmd = %d, index = %d", cmd, index);
  }
  remote_cmd.data = (index << 8) | cmd;
  remote_cmd_pub.publish(remote_cmd);
}

/*
<<<<<<< HEAD
=======
 * update status about device: charge voltage; protector status ... 
 * TODO(cmn): add protector and other status those should be updated on 10Hz
 */
void UpdateDeviceStatus() {
  unsigned int charge_ADC = (g_ultrasonic[22] << 8) | (g_ultrasonic[23] & 0xff);
//  double charge_value = 0.2298 * (charge_ADC - 516);
//  double charge_value = 0.2352 * (charge_ADC - 507);
  double charge_value = 0.2330 * (charge_ADC - 509);
  charge_value = charge_value < 0.0 ? 0.0 : charge_value;
  current_charge_value_= charge_value;
  short current_charge_voltage = static_cast <short>(charge_value * 10.0);
  current_charge_voltage = current_charge_voltage < 100 ? 0 : current_charge_voltage;
  current_charge_voltage = current_charge_voltage > 500 ? 0 : current_charge_voltage;
  charge_voltage_ = current_charge_voltage;
  if (charger_monitor_cmd_ && charge_value > charger_low_voltage_) {
//    charger_monitor_cmd_ = 0;
    timeval tv;
    gettimeofday(&tv, NULL);
    charger_on_time = static_cast<double>(tv.tv_sec) + 0.000001 * tv.tv_usec;
  }
/*
  if (current_charge_voltage > 0 && ++charge_count == 0) {
    charge_voltage_ = current_charge_voltage;
  }
  // do charge voltage filter every 30s
  if (current_charge_voltage > 0 && charge_count > 0) {
    sum_charge_voltage += current_charge_voltage;
    if(charge_count == 300) {
      charge_voltage_ = sum_charge_voltage / 300;
      sum_charge_voltage = 0;
      charge_count = 0;
    }
  } else if (current_charge_voltage == 0) {
    charge_voltage_ = 0;
    sum_charge_voltage = 0;
    charge_count = -1;
  }
*/
  GAUSSIAN_INFO("[wc_chassis] adc_charge = %d, current_charge_value: %lf, current_charge_voltage: %d, charge_voltage: %d",
                 charge_ADC, charge_value, current_charge_voltage, charge_voltage_);

  if(protector_num <= 0){
      return;
  }
  // check protector hit
  unsigned int status = g_ultrasonic[0] | protector_bits;
//  status ^= (0xffff >> (32 - protector_num));
//  GAUSSIAN_INFO("[WC_CHASSIS] enablel protector [%x]!!!", (0xffff >> (32 - protector_num)));
  
  unsigned int temp_hit = NONE_HIT;
  for (int i = 0; i < front_protector_list.size(); ++i) {
    if (!(status & (1 << front_protector_list.at(i)))) {
      temp_hit |= FRONT_HIT;
      GAUSSIAN_ERROR("[WC_CHASSIS] front protector bit[%d] hit!!!", front_protector_list.at(i));
      break;
    }
  }
  for (int i = 0; i < rear_protector_list.size(); ++i) {
    if (!(status & (1 << rear_protector_list.at(i)))) {
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
 /*
  if (protector_service_call) 
    protector_value = NONE_HIT;
  protector_service_call = 0;
  */
  protector_hit = temp_hit;
}

/*
>>>>>>> 5f88541c7068869379ecc941e29853eba6d93456
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
    double battery_value = 0.2318 * (battery_ADC - 516);
    int current_battery_capacity;
    current_battery_capacity = (battery_value - battery_empty_level) / (battery_full_level - battery_empty_level) * 100;
    if(current_battery_capacity < 0) current_battery_capacity = 0;
    if(current_battery_capacity > 100) current_battery_capacity = 100;
    // do battery voltage filter
    ++battery_count;
    if(battery_count == 0) {
      display_battery_capacity = current_battery_capacity;
    } else if(battery_count > 0) {
      sum_battery_capacity += current_battery_capacity;
      if(battery_count == 300) {
        display_battery_capacity = sum_battery_capacity / 300;
        battery_count = 0;
        sum_battery_capacity = 0;
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

    GAUSSIAN_INFO("[wc_chassis] battery_ADC: %d; battery_value: %lf;display_battery_capacity: %d; battery_level_: %d",
                  battery_ADC, battery_value, display_battery_capacity, battery_level_);
//    std::cout << "battery_ADC " << battery_ADC << "; battery_value " << battery_value << "; current_battery_capacity " << current_battery_capacity << "; display_battery_capacity " << display_battery_capacity << " battery_level_" << battery_level_ << std::endl;
    device_value.key = std::string("battery");
    device_value.value = std::to_string(display_battery_capacity);
    device_status.values.push_back(device_value);

    device_value.key = std::string("mileage");
    unsigned int mileage = (unsigned int)((g_chassis_mcu->mileage_left_ + g_chassis_mcu->mileage_left_) / 2);
    device_value.value = std::to_string(mileage);
    device_status.values.push_back(device_value);

    device_pub.publish(device_status);
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

