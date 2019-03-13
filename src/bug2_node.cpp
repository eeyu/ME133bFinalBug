#include "bug2_node.hh"

#define DEBUG 1

pose2D init_pose;
pose2D goal_pose(-5, -5, 0);
double MAngle;
pose2D BF_init_pose;
bool BF_began; // booolean for if robot has left the "starting" BF position

pose2D curr_pose;
double range_min;
double range_max;
std::vector<float> range;
int range_size = 0;
const double clearance = 0.6;  // obstacle proximity to consider as a collision
const double step = 0.1;
const double tolerance_pos = step/2; // 2 points are considered touching if within tolerance
const double tolerance_ang = 0.05; 

bool mode = 0; // 0 = MTG, 1 = BF
bool initialized = 0;


// ----------- FUNCTIONS --------------------
void init() {
  MAngle = atan2(goal_pose.y - init_pose.y, goal_pose.x - init_pose.x);

  while (range_size == 0) {
    ros::spinOnce();
  }
  init_pose.set(curr_pose);
  initialized = 1;
}

geometry_msgs::Twist move_cmd(double forward_move, double turn_move) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = forward_move;
    cmd.angular.z = turn_move;
    return cmd;
}

bool detectCollision() {
  double frontScan = range.at(range_size/2);
  ROS_INFO("frontScan: %f", frontScan);
  if (frontScan <= clearance) {
    return 1;
  }
  return 0;
}

bool detectOnMline() {
  pose2D goal_tr = goal_pose.vec_sub(init_pose);
  pose2D curr_tr = curr_pose.vec_sub(init_pose);
  pose2D proj = goal_tr.vec_scale(curr_tr.vec_dot(goal_tr)/goal_tr.vec_norm()/goal_tr.vec_norm());
  pose2D rej = curr_tr.vec_sub(proj);


  #ifdef DEBUG
    ROS_INFO("curr: (%f, %f)", curr_pose.x, curr_pose.y);
  ROS_INFO("goal: (%f, %f)", goal_pose.x, goal_pose.y);
  ROS_INFO("init: (%f, %f)", init_pose.x, init_pose.y);


  ROS_INFO("M-Line offset: %f", rej.vec_norm());
  #endif
  if (rej.vec_norm() < tolerance_pos) {
    #ifdef DEBUG
    ROS_INFO("ON MLINE");
    #endif
    return 1;

  }
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
#ifdef DEBUG
    ROS_INFO("MTG");
#endif 
   // detects collision to switch modes
    // if not aligned with goal, turn to goal
    // if aligned with goal, move to goal
    detectOnMline();

    // check if aligned with goal
    bool isAligned = (fabs(curr_pose.ang -  MAngle) < tolerance_ang);

    bool isCollision = detectCollision();
    if (isCollision) {
        mode = 1;
        BF_init_pose.set(curr_pose);
        return move_cmd(0,0);
    }

    if (isAligned) {
        return move_cmd(step, 0);
    } else {
        return move_cmd(0,MAngle - curr_pose.ang); 
    }
}

geometry_msgs::Twist BF() {
#ifdef DEBUG
    ROS_INFO("BF");
#endif 
  // Detects if on M line to switch mode
  bool isOnMLine = detectOnMline();
  // Detects if at obstacle start point
  bool isAtStart = false;
  if (curr_pose.vec_distance(BF_init_pose) <= tolerance_pos) {
    isAtStart = true;
  }
  // Determines if the robot has "begun" BF
  // The robot begins on the M line and ends on the M line. 
  // Must differentiate M-line contact between begin and end.
  if (!BF_began && !isAtStart)
    BF_began = true;

  if (isOnMLine && BF_began) {
    mode = 0;
    BF_began = false;
    isAtStart = false;
    return move_cmd(0,0);
  }
  // Else, boundary follow.
  else {
    // Obstacle start point encountered again. Exit with failure.
    if (false) { // TO DO
      printf("Failure!\n");
      ros::shutdown();
    }
    
    // Goal reached.
    else if (curr_pose.vec_distance(goal_pose) < tolerance_pos) {
      printf("Success!\n");
      ros::shutdown();
    }
    
    // M line encountered without obstacle.
    else if (false) { // TO DO
      mode = 0;
    }
  }
  return move_cmd(-1,1);
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  //ROS_INFO("I heard scan: [%f]", msg->ranges[0]);
  if (!initialized) {
    range = scan->ranges;
    range_size = range.size();
    range_min = scan->range_min;
    range_max = scan->range_max;
  } else {
    // Update range data.
    range = scan->ranges;
  }


}

void posCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  // Update orientation (x, y, theta).
  double q0 = odom->pose.pose.orientation.w;
  double q3 = odom->pose.pose.orientation.z;
  double zangle = atan2((2*q0*q3),(1-2*q3*q3));

  curr_pose.set(odom->pose.pose.position.x, odom->pose.pose.position.y, zangle);
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

  /*
  //testing
  #ifdef DEBUG
  ROS_INFO("Testing vectors");
  pose2D p1(0,0,0);
  pose2D p2(0,2,0);
  pose2D p3;
  p3 = p1.vec_add(p2);
  ROS_INFO("norm: %f,", p2.vec_norm());
  ROS_INFO("add: (%f, %f)", p3.x, p3.y);
  p3 = p1.vec_sub(p2);
  ROS_INFO("sub: (%f, %f)", p3.x, p3.y);
  ROS_INFO("dot: %f", p1.vec_dot(p2));
  double dist = p1.vec_distance(p2);
  ROS_INFO("dist:  %f", dist);

  p3.set(1,1,0);
  pose2D d1 = p2.vec_sub(p1);
  pose2D d2 = p3.vec_sub(p1);
    ROS_INFO("norm:  %f", d2.vec_dot(d1)/d1.vec_norm()/d1.vec_norm());

  pose2D proj = d1.vec_scale(d2.vec_dot(d1)/d1.vec_norm()/d1.vec_norm());
  ROS_INFO("proj: (%f, %f)", proj.x, proj.y);
  pose2D distVec = d2.vec_sub(proj);
  ROS_INFO("M-Line offset: %f", distVec.vec_norm());
  #endif
  */

  init();
  while (ros::ok())
  {
    // receive data
   // update information
    // decide on MTG or BF
    // do MTG or BF
    geometry_msgs::Twist cmd;
    cmd = decide_move();
    command_pub.publish(cmd);
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}


