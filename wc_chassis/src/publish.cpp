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
