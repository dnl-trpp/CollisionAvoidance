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
#include <cmath>


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

  ROS_INFO("Ranges size: %lu",msg->ranges.size());
  ROS_INFO("Calculation point cloud");
  try{
    listener->waitForTransform("/base_link", msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(1.0));
    projector.transformLaserScanToPointCloud("/base_link", *msg, cloud, *listener);
  }
  catch(tf::TransformException &ex){
    ROS_ERROR("%s", ex.what());
    return;
  }

  auto obstaclePosition = cloud.points[540];
  float obstacleDistance = sqrt(obstaclePosition.x*obstaclePosition.x+obstaclePosition.y*obstaclePosition.y); //msg->ranges[540];
  ROS_INFO("Obstacle Position(In base_link frame): (%f,%f)",obstaclePosition.x,obstaclePosition.y);
  ROS_INFO("Obstacle Distance: %f",obstacleDistance);

  float currentspeed=sqrt(latestCommand.linear.y*latestCommand.linear.y+latestCommand.linear.x*latestCommand.linear.x);
  float clampedspeed= (currentspeed<5.0)  ? currentspeed : 5.0;

  ROS_INFO("Clamped speed: %f ", clampedspeed);
  if(obstacleDistance < clampedspeed ){
    ROS_INFO("Obstacle in proximity detected!");
    float forceIntensity= (1.0 / obstacleDistance)*0.3* clampedspeed;
    float forceX = -(obstaclePosition.x / obstacleDistance) * forceIntensity;
    float forceY = -(obstaclePosition.y / obstacleDistance) * forceIntensity;
    ROS_INFO("Repulsing force: (%f,%f)",forceX,forceY);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = latestCommand.linear.x+forceX;
    if (signbit(cmd_vel.linear.x)!= signbit(latestCommand.linear.x)) cmd_vel.linear.x = 0;
    cmd_vel.linear.y = latestCommand.linear.y+forceY;
    cmd_vel.linear.z = latestCommand.linear.z;
    cmd_vel.angular  = latestCommand.angular;
    cmd_vel_pub.publish(cmd_vel);
  }
  else{
     cmd_vel_pub.publish(latestCommand);
  }

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