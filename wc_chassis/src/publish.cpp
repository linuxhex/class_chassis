#include "publish.h"
#include "init.h"

extern std::vector<int> g_ultrasonic;
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


void PublishUltrasonic(ros::Publisher *ultrasonic_pubs) {
  publish_ultrasonic(ultrasonic_pubs[0], "ultrasonic0", g_ultrasonic[1]);
  publish_ultrasonic(ultrasonic_pubs[1], "ultrasonic1", g_ultrasonic[2]);
  publish_ultrasonic(ultrasonic_pubs[2], "ultrasonic2", g_ultrasonic[3]);
  publish_ultrasonic(ultrasonic_pubs[3], "ultrasonic3", g_ultrasonic[4]);
  publish_ultrasonic(ultrasonic_pubs[4], "ultrasonic4", g_ultrasonic[5]);
  publish_ultrasonic(ultrasonic_pubs[5], "ultrasonic5", g_ultrasonic[6]);
}

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
//#ifdef DEBUG_PRINT
//  std::cout << "roll = " << roll * 57.6 << "; pitch = " << pitch << ";yaw = " << std::endl;
//#endif
  imu_msg.orientation.x = temp.getX();
  imu_msg.orientation.y = temp.getY();
  imu_msg.orientation.z = temp.getZ();
  imu_msg.orientation.w = temp.getW();
  gyro_pub.publish(imu_msg);
}

void PublishRemoteCmd(ros::Publisher &remote_cmd_pub,unsigned char cmd, unsigned short index) {
  std_msgs::UInt32 remote_cmd;
  if (cmd > 0 && cmd < 12) {
    ROS_INFO("[wc_chassis] get remote cmd = %d, index = %d", cmd, index);
  }
  remote_cmd.data = (index << 8) | cmd;
  remote_cmd_pub.publish(remote_cmd);
}

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


void PublishOdom(tf::TransformBroadcaster* odom_broadcaster,ros::Publisher &odom_pub ) {
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

void PublishYaw(ros::Publisher &yaw_pub){
  std_msgs::Float32 msg;
  msg.data = g_odom_tha * 180.0 / M_PI;
  yaw_pub.publish(msg);
}

