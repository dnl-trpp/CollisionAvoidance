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
#include <stdlib.h>

#define ANGULAR_DEVIATION 2 
const float DEFAULT_MIN_DETECTION_ANGLE = -0.436332;
const float DEFAULT_MAX_DETECTION_ANGLE = 0.436332;

ros::Publisher cmd_vel_pub;
bool commandReceived;
geometry_msgs::Twist latestCommand;
tf::TransformListener* listener;


int get_laser_scan_subset(const sensor_msgs::LaserScan laserScan, sensor_msgs::LaserScan& result,float min_angle,float max_angle){
  //TODO::Add sanity checks on min_angle and max_angle

  //Extract points between min_detection_angle and max_detection_angle
  int min_detection_point= std::abs(laserScan.angle_min - min_angle) / laserScan.angle_increment;
  int max_detection_point= laserScan.ranges.size() - std::abs(laserScan.angle_max - max_angle) / laserScan.angle_increment;

  result.header = laserScan.header;
  result.angle_min = laserScan.angle_min+ laserScan.angle_increment*min_detection_point;
  result.angle_max = laserScan.angle_min+ laserScan.angle_increment*max_detection_point;;
  result.angle_increment = laserScan.angle_increment;
  result.time_increment = laserScan.time_increment;
  result.scan_time = laserScan.scan_time;
  result.range_min = laserScan.range_min;
  result.range_max = laserScan.range_max;
  for(int i=0;i<max_detection_point-min_detection_point+1;i++){
    result.ranges.push_back(laserScan.ranges[i+min_detection_point]);
    result.intensities.push_back(laserScan.intensities[i+min_detection_point]);
  }

  return 1;

}


void safe_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)

{
  //After safe_cmd_vel is received, wait for laser scan
  commandReceived = true;
  latestCommand = *msg;
}

void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (!commandReceived) return; //You need a safe_cmd_vel first
  
  commandReceived = false;

  laser_geometry::LaserProjection projector;
  sensor_msgs::PointCloud cloud;
  sensor_msgs::LaserScan subset;

  //get Params
  float min_detection_angle;
  float max_detection_angle;
  ros::param::param<float>("min_detection_angle", min_detection_angle, DEFAULT_MIN_DETECTION_ANGLE);
  ros::param::param<float>("max_detection_angle", max_detection_angle, DEFAULT_MAX_DETECTION_ANGLE);

  get_laser_scan_subset(*msg,subset,min_detection_angle,max_detection_angle);

  //Get laser scan points in /base_link frame  
  try{
    listener->waitForTransform("/base_link", msg->header.frame_id.c_str(), ros::Time(0), ros::Duration(1.0));
    projector.transformLaserScanToPointCloud("/base_link", subset, cloud, *listener);
  }
  catch(tf::TransformException &ex){
    ROS_ERROR("%s", ex.what());
    return;
  }
  
  
  
  //Get nearest point (nearest Obastacle) and its distance
  auto obstaclePosition = cloud.points[0]; //Assigns right type for auto
  float obstacleDistance = subset.range_max+1; //One more of the biggest possible number
  for(int i = 0;i<cloud.points.size();i++){
    auto newPosition= cloud.points[i];
    float newDistance = sqrt(newPosition.x*newPosition.x+newPosition.y*newPosition.y);
    if(newDistance<obstacleDistance && newDistance > subset.range_min && newDistance < subset.range_max){
      obstaclePosition = newPosition;
      obstacleDistance = newDistance;
    }

  }

  ROS_INFO("Obstacle Position(In base_link frame): (%f,%f)",obstaclePosition.x,obstaclePosition.y);
  ROS_INFO("Obstacle Distance: %f",obstacleDistance);

  //Get speed (used for adjusting force)
  float currentspeed=sqrt(latestCommand.linear.y*latestCommand.linear.y+latestCommand.linear.x*latestCommand.linear.x);
  float clampedspeed= (currentspeed<5.0)  ? currentspeed : 5.0;

  //Apply repulsive force if obstacle is too near
  if(obstacleDistance < clampedspeed && latestCommand.linear.x>0){
    ROS_WARN("Obstacle in proximity detected!");
    //Calculating force intensity and direction
    float forceIntensity= (1.0 / obstacleDistance) *0.3* clampedspeed;
    float forceX = -(obstaclePosition.x / obstacleDistance) * forceIntensity;
    float forceY = -(obstaclePosition.y / obstacleDistance) * forceIntensity;
    ROS_WARN("Applying Repulsive force: (%f,%f)",forceX,forceY);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = latestCommand.linear.x+forceX;
    //Check to avoid a resulting velocity in the opposite direction
    //We want to slow down, not to go back
    if (signbit(cmd_vel.linear.x)!= signbit(latestCommand.linear.x)) cmd_vel.linear.x = 0;
    cmd_vel.linear.y = latestCommand.linear.y+forceY;
    cmd_vel.linear.z = latestCommand.linear.z;
    cmd_vel.angular  = latestCommand.angular;

    if(obstaclePosition.y>0) cmd_vel.angular.z = -forceIntensity * ANGULAR_DEVIATION;
    else if(obstaclePosition.y<0) cmd_vel.angular.z =  forceIntensity * ANGULAR_DEVIATION;
    

  
    cmd_vel_pub.publish(cmd_vel);
  }
  else{ //If we are safe, publish original velocity
     cmd_vel_pub.publish(latestCommand);
  }

}



int main(int argc, char **argv)
{

  ros::init(argc, argv, "collision_avoidance");
  commandReceived = false;

  ros::NodeHandle n;

  //Start listener (used in laser_scan_callback)
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