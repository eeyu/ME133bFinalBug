#include "tangentbug_node.hh"

// #define DEBUG 1
#define PI 3.141593
#define GOAL_TYPE 3

#define DEBUG

//pose2D init_pose;                 // Starting Position
pose2D goal_pose;        // Ultimate goal position
std::vector<pose2D> goal_poses;   // Intermediate goal positions
pose2D curr_pose;

// MF variables
double MAngle;

// BF variables
pose2D BF_init_pose;
bool BF_began = false; // Indicates if robot has started BF.
int BF_searchState = 0; // State of finding closest object.
double BF_startAngle;
double tangentAngleOffset;
double BF_minRadius;

// Laser scanner parameters
double angle_max;
double angle_min;
double angle_incr;
double range_min;
double range_max;
std::vector<float> range;
std::vector<float> intensities;
int range_size = 0;

const double clearance = 0.3;  // obstacle proximity to consider as a collision
const double step = 0.2;
const double tolerance_pos = 0.05; // 2 points are considered touching if within tolerance
const double tolerance_ang = 0.002;
const double SCAN_VELOCITY = 0.7;
double scan_direction = -1; // 1 = ccw, -1 = cw
double fail_ang_step = .5;

int mode = -1; // 0 = MTG, 1 = BF, -1 = scanning
bool initialized = 0;


// ----------- ALGORITHMS --------------------
geometry_msgs::Twist decide_move() {
    geometry_msgs::Twist cmd;
  setMAngle();
  if (goal_poses.size() != 0) {
    if (mode == 0) {
      cmd = MTG();
    } else if (mode == 1) {
      cmd = BF();
    } else if (mode == -1) {
      cmd = SCAN();
    }
    return cmd;
  } else {
    ROS_INFO("SUCCESS");
    return move_cmd(0, 10);
  }
}

//
geometry_msgs::Twist SCAN() {
  ROS_INFO("(((((((((( SCAN ))))))))))");

  if (!isAlignedGoal()) {
    ROS_INFO("aligning");
    return alignToGoal();
  }

  // If final goal not within sight, add intermediate goal
  if (distanceToGoal(curr_pose, 0) > range_max) {
    std::vector<pose2D> inter_goal_cands;
    pose2D cand_pose;
    // Adds desireable points to candidate list
    for (int i = 0; i < range_size - 1; i++) {
      if (intensities.at(i) > .5 && intensities.at(i+1) < .5) {
        cand_pose = range2pose(i, 2);
        cand_pose.setAngle(-1);
        inter_goal_cands.push_back(cand_pose);
      } else if (intensities.at(i) < .5 && intensities.at(i+1) > .5) {
        cand_pose = range2pose(i+1, 2);
        cand_pose.setAngle(1);
        inter_goal_cands.push_back(cand_pose);
      } 
    }
    // Adds the M-Line scan
    cand_pose = range2pose(range_size/2, 2);
    // Sets movement type if line intersects obstacle
    if (intensities.at(range_size/2) > .5) 
      cand_pose.setAngle(1);
    else
      cand_pose.setAngle(0);
    ROS_INFO("MLINE OBSTACLE STATE: %f", cand_pose.ang);
    
    inter_goal_cands.push_back(cand_pose);
    for (pose2D posei : inter_goal_cands) {
      printPose(posei, "candidates");
    }

    // Determines the minimum
    double minDist = distanceToGoal(inter_goal_cands.front(), 0);
    pose2D minPose = inter_goal_cands.front();
    double currDist;
    for (pose2D posei : inter_goal_cands) {
      currDist = distanceToGoal(posei, 0);
      if (currDist < minDist) {
        minDist = currDist;
        minPose = posei;
      }
    }

    // Push best goal to list
    printPose(minPose, "Chosen");
    goal_poses.push_back(minPose);
  }

  // Switch to MTG 
  switchToMTG();
  return move_cmd(0,0);
}

  /*
   * Moves to goal until obstacle encountered or goal reached.
   *
   */
geometry_msgs::Twist MTG() {
#ifdef DEBUG
    ROS_INFO("<><><><><><><><> MTG <><><><><><><><>");
    printPose(goal_poses.back(), "Current Goal");
    ROS_INFO("Mangle %f", MAngle);
#endif

  // Checks if a goal has been reached
  if (isAtGoal(1)) {
    if (goal_poses.back().ang != 0) // Goal was BF
      switchToBF();
    else // Goal was MTG
      switchToSCAN();

    ROS_INFO("At inter goal! Changing to: %i", mode);
    goal_poses.pop_back();
    return move_cmd(0,0);
  }
/*

  // Checks if robot encountered an object. Change mode to BF   
  if (detectCollision()) {
    ROS_INFO(" Collision! Following Boundary");
    switchToBF();
    goal_poses.pop_back();
    return move_cmd(0,0); // No move.
  }
  */

  // Align and move robot along M line.
  if (!isAlignedGoal()) 
    return alignToGoal();
  else
    return move_cmd(step, 0);
}




geometry_msgs::Twist BF() {
#ifdef DEBUG
    ROS_INFO("## ## ## ## ## ## BF ## ## ## ## ## ##");
    ROS_INFO("Min Dist to Goal: %f", BF_minRadius);
#endif 
  // Follows boundary until boundary circumnavigated or M line found.
  if (!isAtPosition(BF_init_pose) && !BF_began)
    BF_began = true;

  // Things to check if BG has begun
  if (BF_began) {
    // Robot has returned to initial collision point
    if (isAtPosition(BF_init_pose)) {
      ROS_INFO("FAILURE: AT START.");
      return move_cmd(0, 0);
    }

    // Robot has traced to a position closer to goal
    double dist2goal = distanceToGoal(curr_pose, 0);
    if (dist2goal < BF_minRadius) {
      ROS_INFO("Checking M Line Blocked");
      // Switch to SCAN if goal is not blocked
      if (isFinalGoalBlocked()) {
        ROS_INFO("BLOCKED.");
        BF_minRadius = dist2goal;
      } else {
        ROS_INFO("NOT BLOCKED");
        mode = -1;
        return move_cmd(0, 0);
      }
    }
  }

  // Otherwise, does BF
  ROS_INFO("Stepping");
  return BF_step();
}

geometry_msgs::Twist BF_step() {
  // Search for closest obstacle.
  if (BF_searchState == 0)
    BF_searchObject();

  // Move tangent to object.
  double currAngleOffset = limitAngle(curr_pose.ang - BF_startAngle);
  double error =  limitAngle(tangentAngleOffset - currAngleOffset);

  ROS_INFO("Error: %f", error);
  // Align Robot with tangent angle
  if (fabs(error) >= tolerance_ang) {
    ROS_INFO ("Aligning");
    ROS_INFO ("MAngle: %f", MAngle);
    return move_cmd(0, 2*error);
  }
    
  // Step if aligned
  else {
    // Reset states.
    BF_searchState = 0;
    ROS_INFO("Moved");

    // Move forward.
    return move_cmd(3 * step, 0);   
  }
}

geometry_msgs::Twist BF_searchObject() {
   // Searches for min range through entire scan data. This is the tangency point
  int range_idx = std::min_element(range.begin(), range.end()) - range.begin();
  // Converts the index to angle in robot frame
  tangentAngleOffset = (range_idx * 1.0/range_size) * (angle_max - angle_min) + angle_min;
  // Find the tangency angle to tangency point
  tangentAngleOffset += -scan_direction * PI/2;
  // Store obstacle range
  double minObsRange = *std::min_element(range.begin(), range.end());
  // Controller to maintain range
  double k = 2;
  tangentAngleOffset += scan_direction * k*(minObsRange - clearance);

  // Account for angle being -PI to PI if necessary.
  tangentAngleOffset = limitAngle(tangentAngleOffset);
  // Convert to convenient form
  if (tangentAngleOffset - curr_pose.ang > PI)
    tangentAngleOffset -= 2*PI;
  else if (tangentAngleOffset - curr_pose.ang < -PI)
    tangentAngleOffset += 2*PI;

  BF_startAngle = curr_pose.ang;
  BF_searchState++;
  return move_cmd(0, 0);
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if (!initialized) { // Initialize if necessary
    range = scan->ranges;
    range_size = range.size();
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_max = scan->angle_max;
    angle_min = scan->angle_min;
    angle_incr = scan->angle_increment;
    intensities = scan->intensities;
  } else {
    // Update range data.
    range = scan->ranges;
    intensities = scan->intensities;

  }
}

void posCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  // Update orientation (x, y, theta).
  double q0 = odom->pose.pose.orientation.w;
  double q3 = odom->pose.pose.orientation.z;
  double zangle = atan2((2*q0*q3),(1-2*q3*q3));

  curr_pose.set(odom->pose.pose.position.x, odom->pose.pose.position.y, zangle);
  ROS_INFO("curr: (%f, %f, %f)", curr_pose.x, curr_pose.y, curr_pose.ang);
}

// ----------- FUNCTIONS --------------------
void init() {
  while (range_size == 0) {
    ros::spinOnce();
  }
  //init_pose.set(curr_pose);
  initialized = 1;
    //ROS_INFO("init: (%f, %f)", init_pose.x, init_pose.y);
  if (GOAL_TYPE == 1)
    goal_pose.set(3,3,0);
  else if (GOAL_TYPE == 2) 
    goal_pose.set(3, -3, 0);
  else if (GOAL_TYPE == 3) 
    goal_pose.set(-3, -3, 0);
  else if (GOAL_TYPE == 4) 
    goal_pose.set(-3, 3, 0);
  else
    goal_pose.set(0, 0, 0);

  goal_poses.push_back(goal_pose);
 // setMAngle();
  ROS_INFO("Mangle: %f", MAngle);
}

void switchToBF() {
  mode = 1;
  scan_direction = (int) goal_poses.back().ang;
  BF_init_pose.set(curr_pose);
  BF_minRadius = distanceToGoal(BF_init_pose, 0);
  BF_began = false;
}

void switchToMTG() {
 // setMAngle();
  mode = 0;
}

void switchToSCAN() {
  mode = -1;
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
  return range_val/num;
}

bool detectCollision() {
  double detectionRange = PI/6;
  int indexRange = (int) (detectionRange/angle_incr);
  int range_idx = std::min_element(range.begin() + range_size/2 - indexRange/2, 
                                  range.begin() + range_size/2 + indexRange/2) - range.begin();
  double minRange = range.at(range_idx);
  if (minRange <= clearance)
    return 1;
  return 0;
}

// true if obstacle within range
bool detectObstacle() {
  double frontScan = intensities[range_size/2];
  ROS_INFO("Obstacle detected?: %f", frontScan);
  if (frontScan)
    return 1;
  return 0;
}



// Converts 1 piece of laser scan data to a position in world frame
pose2D range2pose(int ind, int useClearance) {
  double range_val = range.at(ind) - useClearance * clearance;
  // Converts the index to angle in robot frame
  double angleOffset = (ind * 1.0/range_size) * (angle_max - angle_min) + angle_min; 
  // convert angle to world frame
  angleOffset += curr_pose.ang;
  // creates the offset vector
  pose2D vec_offset(range_val * cos(angleOffset), range_val * sin(angleOffset), 0);
  // adds offset to current position
  return curr_pose.vec_add(vec_offset);

}

bool isAlignedGoal() {
//  ROS_INFO("Checking Alignment");
 //   printPose(curr_pose, "Current Angle");

  if (fabs(curr_pose.ang - MAngle) < tolerance_ang) 
    return 1;
  return 0;    
}


geometry_msgs::Twist alignToGoal() {
  double error = limitAngle(MAngle - curr_pose.ang);
  return move_cmd(0, 2*error); 
}

// checks distance to goal position. 
// 0 = final goal, 1 = current goal
double distanceToGoal(pose2D pose, int goalType) {
  pose2D tempPose;
  if (goalType == 0)
    tempPose = goal_pose;
  else
    tempPose = goal_poses.back();
  ROS_INFO("dist2goal: %f", pose.vec_distance(tempPose));
  return pose.vec_distance(tempPose);
}

// detects if current position is closer to final goal than another position
bool isCloserToGoal(pose2D compPose) {
  if (distanceToGoal(curr_pose, 0) < distanceToGoal(compPose, 0)) 
    return 1;
  return 0;
}

// checks if currently at a goal position. 
// 0 = final goal, 1 = current goal
bool isAtGoal(int goalType) {
  if (distanceToGoal(curr_pose, goalType) < tolerance_pos)
    return 1;
  return 0;
}

// Checks if at a position
bool isAtPosition(pose2D pose2) {
  if (curr_pose.vec_distance(pose2) < tolerance_pos)
    return 1;
  return 0;
}

bool isFinalGoalBlocked() {
  // Angle of goal relative to front of robot
  double scanAngle = limitAngle(curr_pose.ang - MAngle);
  // converting to index position of angle
  double indexAngle = angle_max - scanAngle;
  int index = (int) (indexAngle/angle_incr);
//  ROS_INFO("scan angle index: %i", index);
  // Check if any obstacle is near the bot blocking the goal
  int width = 50;
  for (int i = -width/2; i <= width/2; i++) {
  //  ROS_INFO("scan range: %f", range[index+i]);
    if (range[index + i] < 2*clearance)
      return 1;
  }
  return 0;

}

double limitAngle(double angle) {
  if (angle > PI)
    return angle - 2*PI;
  else if (angle < - PI)
    return angle + 2*PI;
  else 
    return angle;
}

// rosprints a pose
void printPose(pose2D pose, std::string name) {
    ROS_INFO("%s: (%f, %f, %f)", name.c_str(), pose.x, pose.y, pose.ang);
}

inline void setMAngle() {
  MAngle = atan2(goal_poses.back().y - curr_pose.y, goal_poses.back().x - curr_pose.x);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  // Publishes commands.
  ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000);
  // Subscribes to laser data and position data
  ros::Subscriber subLaser = n.subscribe<sensor_msgs::LaserScan>("scan", 1000, scanCallback);
  ros::Subscriber subPos = n.subscribe<nav_msgs::Odometry>("odom", 1000, posCallback);
  
  // Set rate to 10 Hz.
  ros::Rate loop_rate(10);

  init();

  while (ros::ok())
  {
    // receive data
    // update information
    // decide action
    // do action
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


