#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include “nav_msgs/Odometry.h”
#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>


float64  init_pose[3];
float64  goal_pose[2] = {3, 3};
float64 orientation [3];
float32 range_min;
float32 range_max;
float32[] range;
const float32 clearance = 0.6;  // obstacle proximity to consider as a collision
const float32 step = 0.1;
const float32 tolerance_pos = step/2; // 2 points are considered touching if within tolerance
const float32 tolerance_ang = 0.05; 

bool mode = 0; // 0 = MTG, 1 = BF

geometry_msgs::Twist move_cmd(double forward_move, double turn_move) {
    cmd.linear.x = forward_move;
    cmd.angular.z = turn_move;
    return cmd;
}

bool detectCollision() {
  return 0;
}

geometry_msgs::Twist decide_move() {
    geometry_msgs::Twist cmd;
    // execute MTG or BG depending on mode
    // also report failure or success

    if (mode == 0) {
        cmd = MTG();
    } else {
        cmd = BF();
    }
    return cmd;
}

geometry_msgs::Twist MTG() {
   // detects collision to switch modes
    // if not aligned with goal, turn to goal
    // if aligned with goal, move to goal

    // check if aligned with goal
    float64 MAngle = tan2(goal_pose[1] - init_pose[1], goal_pose[0] - init_pose[0]);
    float64 currAngle = orientation[2];
    bool isAligned = (abs(currAngle - MAngle) < tolerance_ang); // TODO

    bool isCollision = detectCollision();
    if (isCollision) {
        mode = 1;
        return move_cmd(0,0);
    }

    if (isAligned) {
        return move_cmd(step, 0);
    } else {
        return move_cmd(0,currAngle - MAngle); // TODO Change angle appropriately
    }
}

geometry_msgs::Twist BF() {
    // Detects if on M line to switch mode
    if (isOnMLine) {
    }
    // Else, boundary follow.
    else {
      // Obstacle start point encountered again. Exit with failure.
      // Goal reached.
      // M line encountered without obstacle.
    }
    bool isOnMLine = 0; // TODO
    if (isOnMLine) {
        mode = 0;
        return move_cmd(0,0);
    }
    return move_cmd(0,0);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("I heard scan: [%f]", msg->ranges[0]);
  
  // Update range data.
  sensor_msgs::LaserScan scan = msg;
  range = scan.ranges;
  range_min = scan.range_min;
  range_max = scan.range_max;
}

void posCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("I heard pose: [%f]", msg->ranges[0]);

  // Update orientation (x, y, theta).
 nav_msgs::Odometry odom = msg;
  orientation = {odom.pose.pose.position.x, odom.pose.pose.position.y, odom.twist.twist.angular.z};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  // Publishes commands
  ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000);
  ros::Rate loop_rate(10);

  // Subscribes to laser data and position data
  ros::Subscriber subLaser = n.subscribe<sensor_msgs::LaserScan>("scan", 1000, scanCallback);
  ros::Subscriber subPos = n.subscribe<nav_msgs::Odometry>("odom", 1000, posCallback);

  while (ros::ok())
  {
    // receive data
   // update information
    // decide on MTG or BF
    // do MTG or BF

    geometry_msgs/Twist cmd;
    cmd = decide_move();
    command_pub.publish(cmd);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}


