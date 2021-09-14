# Collision Avoidance ROS Package
This is a simple Collision Avoidance implementation for ROS. 
It reads from two topics:
* `base_scan` of type `sensor_msgs/LaserScan`
* `safe_cmd_vel` of type `geometry_msgs/Twist`

Given a  velocity as input, this node uses the laser scan to locate obstacles and adjust the velocity accordingly.
It then proceeds to publish the calculated velocity on `cmd_vel` of type `geometry_msgs/Twist`

![slow_speed](Slow_speed.mp4)

# How to run 

* Copy the contents of this repo in a folder inside the `src` folder of your ROS workspace
* from your workspacke build with `catkin_make`
* run with `rosrun labiagi_proj collision_avoidance`

# Test this node

The simplest way to test this node is by using stage ros and a controller node that publishes `Twist` messages such as `teleop_twist_keyboard`.

Install the controller using:
```
sudo apt-get install ros-melodic-teleop-twist-keyboard
```
First, run the collision_avoidance node with:
```
rosrun labiagi_proj collision_avoidance
```
Second, the controller with:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=safe_cmd_vel
```
>Note: By default `teleop_twist_keyboard` publishes on `cmd_vel`. Because we want our commands filtered by the collision_avoidance node we use [ROS topic remapping](http://wiki.ros.org/Remapping%20Arguments) (`cmd_vel:=safe_cmd_vel`)

Now run ros_stage with:
```
rosrun stage_ros stageros <worldfile>
```
>Note: [cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world](https://gitlab.com//grisetti/labiagi_2020_21/-/raw/master/workspaces/srrg2_labiagi/src/srrg2_navigation_2d/config/cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world?inline=false) was used for testing

Make sure the controller terminal is selected. Now you can move using:
* `j` turn left
* `i` move forward
* `l` turn right
* `,` move backwards



