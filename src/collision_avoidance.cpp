#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>

ros::Publisher cmd_vel_pub;

void safe_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("Safe Cmd Vel: [x:%f,y:%f,z:%f]", msg->linear.x, msg->linear.y,msg->linear.z);
}

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("Laser Scan: [%s]", msg->header.frame_id.c_str());
}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "collision_avoidance");

  ros::NodeHandle n;

  //Subscribe to safe_cmd_vel topic
  ros::Subscriber safe_cmd_vel_sub = n.subscribe("safe_cmd_vel", 1000, safe_cmd_vel_callback);
  //Subscribe to laser scan topic
  ros::Subscriber laser_scan_sub = n.subscribe("base_scan", 1000, laser_scan_callback);

  //Pubblish on cmd_vel topic
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  ros::spin();

  return 0;
}