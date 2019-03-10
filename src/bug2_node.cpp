#include "bug2_node.hh"

#define DEBUG 1
double  init_pose[2];
double  goal_pose[2] = {-5, -5};
double orientation [3];
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

void init() {
  init_pose[0] = orientation[0];
  init_pose[1] = orientation[1];
  while (range_size == 0) {
    ros::spinOnce();
  }
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

    // check if aligned with goal
    double MAngle = atan2(goal_pose[1] - init_pose[1], goal_pose[0] - init_pose[0]);
    double currAngle = orientation[2];

    bool isAligned = (fabs(currAngle -  MAngle) < tolerance_ang); // TODO

    bool isCollision = detectCollision();
    if (isCollision) {
        mode = 1;
        return move_cmd(0,0);
    }

    if (isAligned) {
        return move_cmd(step, 0);
    } else {
        return move_cmd(0,MAngle - currAngle); 
    }
}

geometry_msgs::Twist BF() {
#ifdef DEBUG
    ROS_INFO("BF");
#endif 
    // Detects if on M line to switch mode
    bool isOnMLine = 0;
    if (isOnMLine) {
      mode = 0;
      return move_cmd(0,0);
    }
    // Else, boundary follow.
    else {
      // Obstacle start point encountered again. Exit with failure.
      // Goal reached.
      // M line encountered without obstacle.
    }
    return move_cmd(0,1);
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
 //nav_msgs::Odometry odom = msg;
  orientation[0] = odom->pose.pose.position.x;
  orientation[1] = odom->pose.pose.position.y;
  //orientation[2] = odom->twist.twist.angular.z;
  double q0 = odom->pose.pose.orientation.w;
  double q3 = odom->pose.pose.orientation.z;
  double zangle = atan2((2*q0*q3),(1-2*q3*q3));
  orientation[2] = zangle;
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


