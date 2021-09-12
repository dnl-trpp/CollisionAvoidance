#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>


ros::Publisher cmd_vel_pub;
bool commandReceived;
geometry_msgs::Twist latestCommand;
tf::TransformListener* listener;


void safe_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  commandReceived = true;
  //Copy shared pointer (Object will not be destroyed)
  latestCommand = *msg;
}

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (!commandReceived) return;
  
  commandReceived = false;

  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud cloud;

  ROS_INFO("Calculation point cloud");
  //listener.waitForTransform("/base_link", msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(10.0));
  projector.transformLaserScanToPointCloud("/base_link", *msg, cloud, *listener);

  for(auto& point : cloud.points){
    ROS_INFO("x:%f,y:%f,z:%f",point.x,point.y,point.z);
  }

  cmd_vel_pub.publish(latestCommand);

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "collision_avoidance");
  commandReceived = false;

  ros::NodeHandle n;

  tf::TransformListener lr(ros::Duration(10));
  listener=&lr;
  //Subscribe to safe_cmd_vel topic
  ros::Subscriber safe_cmd_vel_sub = n.subscribe("safe_cmd_vel", 1000, safe_cmd_vel_callback);
  //Subscribe to laser scan topic
  ros::Subscriber laser_scan_sub = n.subscribe("base_scan", 1000, laser_scan_callback);

  //Pubblish on cmd_vel topic
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  ros::spin();

  return 0;
}