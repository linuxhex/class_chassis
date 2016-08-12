#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void DoNavigationCallback(const geometry_msgs::Twist& Navigation_msg) {
  std::cout << "heard" << std::endl;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/cmd_vel", 10, DoNavigationCallback);

  ros::spin();
  return 0;
}
