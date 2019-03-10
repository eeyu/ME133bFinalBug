#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include "math.h"
#include <vector>

geometry_msgs::Twist move_cmd(double forward_move, double turn_move);

bool detectCollision();

geometry_msgs::Twist decide_move();

geometry_msgs::Twist MTG();

geometry_msgs::Twist BF();

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

void posCallback(const nav_msgs::Odometry::ConstPtr& msg);



