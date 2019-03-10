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




struct pose2D {
  double x;
  double y;
  double ang;

  pose2D() {x = 1; y = 0; ang = 0;}

  pose2D(double nx, double ny, double nang) {x = nx; y = ny; ang = nang;}

  void set(double nx, double ny, double nang) {x = nx; y = ny; ang = nang;}

  void set(pose2D p2) {x = p2.x; y = p2.y; ang = p2.ang;}

  double vec_norm() {
    return sqrt(x*x+y*y);
  }

  pose2D vec_sub(pose2D p2) {
    pose2D p_new(x - p2.x, y - p2.y, 0);
    return p_new;
  }

  pose2D vec_add(pose2D p2) {
    pose2D p_new(x + p2.x, y + p2.y, 0);
    return p_new;
  }
  
  double vec_dot(pose2D p2) {
    return this->x * p2.x + this->y * p2.y;
  }

  double vec_distance(pose2D p2) {
    pose2D p3 = this->vec_sub(p2);
    return p3.vec_norm();
  }

  pose2D vec_scale(double s) {
    pose2D p2((this->x)*s, (this->y)*s, this->ang);
    return p2;
  }

};
