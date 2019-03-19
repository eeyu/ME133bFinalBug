#include "bug2_node.hh"

// #define DEBUG 1
#define PI 3.141593
#define GOAL_TYPE 3
#define DEBUG

pose2D init_pose;
pose2D goal_pose;
pose2D curr_pose;

// MF variables
double MAngle;

// BF variables.
pose2D BF_init_pose;
bool BF_began = false; // Indicates if robot has started BF.
int BF_searchState = 0; // State of finding closest object.
int BF_moveState = 0; // State of moving tangentially to object.
double BF_startAngle;
double minObsRange, minObsRangeAngle; // Range and angle to closest object.
double tangentAngle; // Angle that is parallel to tangent.
double tangentAngleOffset;
int c = 0;
ros::Time BF_searchStartTime;
int SCAN_TIME = 2.5; // Seconds to scan.
double BF_temp_X; // Quickly hashed debug to solve MLine blocked issue. Used to check if 
				  // position has changed
bool MTGpossible = true;



// Laser scanner parameters
double angle_max;
double angle_min;
double angle_incr;
double range_min;
double range_max;
std::vector<float> range;
int range_size = 0;

const double clearance = 0.4;  // obstacle proximity to consider as a collision
const double step = 0.2;
const double tolerance_pos = 0.05; // 2 points are considered touching if within tolerance
const double tolerance_ang = 0.002;
const double SCAN_VELOCITY = 0.7;
double scan_direction = -1; // 1 = ccw, -1 = cw
double fail_ang_step = .5;

bool mode = 0; // 0 = MTG, 1 = BF
bool initialized = 0;


// ----------- ALGORITHMS --------------------

geometry_msgs::Twist MTG() {
#ifdef DEBUG
  //  ROS_INFO("MTG");
#endif
  /*
   * Moves to goal until obstacle encountered or goal reached.
   *
   */
//ROS_INFO("M Angle: %f", MAngle);
  bool isCollision = detectCollision(); // Detect collision.

  // Robot encounters an object.
  // Change mode to BF and save initial collision point.
  if (isCollision) {
  	ROS_INFO("BUMP");
  	  ROS_INFO(" ");
  ROS_INFO("## ## ## ## ## ## BF ## ## ## ## ## ##");
    mode = 1;
   	BF_temp_X = curr_pose.x;
    BF_init_pose.set(curr_pose);
    BF_began = false;
    MTGpossible = true;


    return move_cmd(0,0); // No move.
  }
  // No obstacle encountered.
  // Align and move robot along M line.
  else 
    return align();
}




geometry_msgs::Twist BF() {
#ifdef DEBUG
  //  ROS_INFO("BF");
#endif 
  // Follows boundary until boundary circumnavigated or M line found.
  if ((curr_pose.vec_distance(BF_init_pose) > tolerance_pos) && !BF_began) {
    BF_began = true;
  }

  // Robot has started BF around the obstacle.
  if (BF_began) {

    // Check if robot at start.
    bool isAtStart = curr_pose.vec_distance(BF_init_pose) <= tolerance_pos;
    // Check if MTG is possible. Initialize as true.
    
    // If robot returns to initial collision point after starting BF,
    // exit with failure.
    if (fabs(BF_temp_X - curr_pose.x) < 2*tolerance_pos)
    	MTGpossible = true;
    BF_temp_X = curr_pose.x;
    if (isAtStart) {
      ROS_INFO("FAILURE: AT START.");
      return move_cmd(0, 0);
    }
    // If robot finds M line without an obstacle,
    // switch to MTG if possible.
    else if (detectOnMline() && MTGpossible) {
    	ROS_INFO("ON MLINE");
    	//ROS_INFO("ON M-Line. Checking if blocked");
      geometry_msgs::Twist moveAlign = align();
      // Turn to face M line.
      if (moveAlign.angular.z > 0) {
        return moveAlign;
      }
      
      // If M line is blocked, continue BF.
      if (detectCollision()) {
        ROS_INFO("M-LINE BLOCKED. Continuing.");
      //  MTGpossible = false;
      //  fail_ang_step *= -1;
        //return move_cmd(0, fail_ang_step);
      }
      // Else, switch to MTG.
      else {
      	ROS_INFO(" ");
  ROS_INFO("<><><><><><><><> MTG <><><><><><><><>");
        mode = 0;
        return move_cmd(0, 0);
      }
      
    }
  }
  return BF_step();
}

geometry_msgs::Twist BF_step() {
  // 0: Initialize search for closest obstacle.
  if (BF_searchState == 0) {
    BF_startAngle = curr_pose.ang; // Start angle

    tangentAngle = curr_pose.ang;
    minObsRange = range.at(range_size / 2);

 //   ROS_INFO("START ANGLE: %f", BF_startAngle);

    // Advance state.
    BF_searchState = 1;
    BF_searchStartTime = ros::Time::now();
    return move_cmd(0, 0);
  }

  // 1: Search for the closest object
  else if (BF_searchState == 1) {
    return BF_searchObject();
  }


  // 2: Move tangent to object.
  else {
  //  ROS_INFO("MIN RANGE: %f", minObsRange);
   // ROS_INFO("TANGENT ANGLE OFFSET: %f", tangentAngleOffset);
   // ROS_INFO("CURRENT OFFSET: %f", (curr_pose.ang - BF_startAngle));
    double currAngleOffset = (curr_pose.ang - BF_startAngle);
    if (currAngleOffset > PI) currAngleOffset -= 2*PI;
    if (currAngleOffset < -PI) currAngleOffset += 2*PI;
    double error =  tangentAngleOffset - currAngleOffset;
  //  ROS_INFO("ERROR: %f", error);

    // Robot not aligned with tangent angle.
    if (fabs(error) >= tolerance_ang)
      return move_cmd(0, 2*error);
      // Robot aligned with tangent angle. Move forward.
    else {
      // Reset states.
      BF_searchState = 0;
      BF_moveState = 0;

    //  ROS_INFO("--MOVED--");

      // Move forward.
      return move_cmd(3 * step, 0);
    }
  }
}

geometry_msgs::Twist BF_searchObject() {
  /*
  double currRange = scan_center_avg(5);

  // Update angle of closest object found if necessary.
  if ((currRange > minObsRange) && (currRange < 2*clearance)) {
    BF_searchState++;
    // Find angle that is parallel to the tangent of the object.
    // adjusts accordingly to keep near boundary
    double k = 5;
    tangentAngle += scan_direction * k*(currRange - clearance);
    // Account for angle being -PI to PI if necessary.
    if (tangentAngle > PI / 2)
      tangentAngle -= 3 * PI / 2;
    else
      tangentAngle += PI / 2;
    ROS_INFO("FOUND");
  }
  if (currRange < minObsRange) {
    tangentAngle = curr_pose.ang;
    minObsRange = currRange;
  }

  ROS_INFO("MIN RANGE: %f", minObsRange);
  ROS_INFO("CURR RANGE: %f", currRange);


  ROS_INFO("MIN RANGE ANGLE: %f", tangentAngle);

  // Limit search time
  //  ros::Time currTime = ros::Time::now();
   // if (currTime.sec - BF_searchStartTime.sec > SCAN_TIME) {
    //  ROS_INFO("TIME");
     // BF_searchState++;
   // }

  return move_cmd(0, scan_direction * SCAN_VELOCITY);
  */

  // Searches for min range through entire scan data. This is the tangency point
  int range_idx = std::min_element(range.begin(), range.end()) - range.begin();
  // Converts the index to angle in robot frame
  tangentAngleOffset = (range_idx * 1.0/range_size) * (angle_max - angle_min) + angle_min;
  // Find the tangency angle to tangency point
  tangentAngleOffset += -scan_direction * PI/2;
  // Store obstacle range
  minObsRange = *std::min_element(range.begin(), range.end());
  // Controller to maintain range
  double k = 2;
  tangentAngleOffset += scan_direction * k*(minObsRange - clearance);

  // Converts the angle to world frame
  //tangentAngle = curr_pose.ang + tangentAngleOffset;
  //ROS_INFO("TA world frame: %f", tangentAngle);

  // Account for angle being -PI to PI if necessary.
  if (tangentAngle > PI)
    tangentAngle -= 2*PI;
  else if (tangentAngle < -PI)
    tangentAngle += 2*PI;
  // Convert to convenient form
  if (tangentAngle - curr_pose.ang > PI)
    tangentAngle -= 2*PI;
  else if (tangentAngle - curr_pose.ang < -PI)
    tangentAngle += 2*PI;

  BF_startAngle = curr_pose.ang;
  BF_searchState++;
  return move_cmd(0, 0);
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  //ROS_INFO("I heard scan: [%f]", msg->ranges[0]);
  if (!initialized) {
    range = scan->ranges;
    range_size = range.size();
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_max = scan->angle_max;
    angle_min = scan->angle_min;
    angle_incr = scan->angle_increment;
  //    ROS_INFO("m: %f", angle_min);

 // ROS_INFO("M: %f", angle_max);


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
    //ROS_INFO("curr: (%f, %f, %f)", curr_pose.x, curr_pose.y, curr_pose.ang);

}

// ----------- FUNCTIONS --------------------
void init() {
  while (range_size == 0) {
    ros::spinOnce();
  }

  if (GOAL_TYPE == 1)
    goal_pose.set(3,3,0);
  else if (GOAL_TYPE == 2) 
    goal_pose.set(3, -3, 0);
  else if (GOAL_TYPE == 3) 
    goal_pose.set(-3, -3, 0);
  else if (GOAL_TYPE == 4) 
    goal_pose.set(-2, 2, 0);
  else
    goal_pose.set(0, 0, 0);
  init_pose.set(curr_pose);
  initialized = 1;
  MAngle = atan2(goal_pose.y - init_pose.y, goal_pose.x - init_pose.x);
      ROS_INFO("Goal: (%f, %f)", goal_pose.x, goal_pose.y);

    ROS_INFO("<><><><><><><><> MTG <><><><><><><><>");

 // ROS_INFO("Mangle %f", MAngle);
  //ROS_INFO("init: (%f, %f)", init_pose.x, init_pose.y);

}

geometry_msgs::Twist move_cmd(double forward_move, double turn_move) {
    geometry_msgs::Twist cmd;
    cmd.linear.x = forward_move; // Unit: m/s
    cmd.angular.z = turn_move; // Unit: rad/s
    return cmd;
}

double scan_center_avg(int num) {
  double range_val = 0;
  for (int i = -num/2; i<=num/2; i++) {
    range_val += range.at(range_size / 2 + i);
  }
  return range_val/5;
}

bool detectCollision() {
  double frontScan = range.at(range_size / 2);
//  minObsRange = *std::min_element(range.begin(), range.end());
 // ROS_INFO("frontScan: %f", frontScan);
  if (frontScan <= clearance) {
    return 1;
  }
  return 0;
}

bool detectOnMline() {
  pose2D goal_tr = goal_pose.vec_sub(init_pose);
  pose2D curr_tr = curr_pose.vec_sub(init_pose);
  pose2D proj = 
    goal_tr.vec_scale(curr_tr.vec_dot(goal_tr)/goal_tr.vec_norm()/goal_tr.vec_norm());
  pose2D rej = curr_tr.vec_sub(proj);

  MAngle = atan2(goal_pose.y - curr_pose.y, goal_pose.x - curr_pose.x);


  #ifdef DEBUG
//  ROS_INFO("curr: (%f, %f)", curr_pose.x, curr_pose.y);
//  ROS_INFO("goal: (%f, %f)", goal_pose.x, goal_pose.y);
//  ROS_INFO("init: (%f, %f)", init_pose.x, init_pose.y);
 // ROS_INFO("M-Line offset: %f", rej.vec_norm());
  #endif
  
  if (rej.vec_norm() < tolerance_pos) {
    #ifdef DEBUG
 //  ROS_INFO("ON MLINE");
    #endif

    return 1;
  }
  return 0;
}

geometry_msgs::Twist align() {
  // Move forward if aligned with M line.
  if (fabs(curr_pose.ang -  MAngle) < tolerance_ang)
    return move_cmd(step, 0);
  // Else turn to align with M line.
  else 
    return move_cmd(0, MAngle - curr_pose.ang); 
}

geometry_msgs::Twist decide_move() {
    geometry_msgs::Twist cmd;
    // execute MTG or BG depending on mode
    // also report failure or success

    if (curr_pose.vec_distance(goal_pose) <= tolerance_pos) {
return move_cmd(0, 10);
ROS_INFO("SUCCESS");
//      ros::shutdown();
    }
    if (mode == 0) {
        cmd = MTG();
    } else {
        cmd = BF();
    }
    return cmd;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  // Publishes commands.
  ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000);

  // Set rate to 10 Hz.
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

    if (!ros::ok()) ROS_INFO("ERROR 1!");
  }

  if (!ros::ok()) ROS_INFO("ERROR 2!");

  return 0;
}


