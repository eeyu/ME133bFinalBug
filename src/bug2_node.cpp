#include "bug2_node.hh"

// #define DEBUG 1
#define PI 3.141593

pose2D init_pose;
pose2D goal_pose(5, 5, 0);
double MAngle;

// BF variables.
pose2D BF_init_pose;
bool BF_began = false; // Indicates if robot has started BF.
int BF_searchState = 0; // State of finding closest object.
int BF_moveState = 0; // State of moving tangentially to object.
double startAngle;
double minRange, minRangeAngle; // Range and angle to closest object.
double tangentAngle; // Angle that is parallel to tangent.
int c = 0;

pose2D curr_pose;
double range_min;
double range_max;
std::vector<float> range;
int range_size = 0;

const double clearance = 0.8;  // obstacle proximity to consider as a collision
const double step = 0.5;
const double tolerance_pos = 0.1; // 2 points are considered touching if within tolerance
const double tolerance_ang = 0.02;
const double SCAN_VELOCITY = -0.4;

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
    cmd.linear.x = forward_move; // Unit: m/s
    cmd.angular.z = turn_move; // Unit: rad/s
    return cmd;
}

bool detectCollision() {
  double frontScan = range.at(range_size / 2);
  ROS_INFO("frontScan: %f", frontScan);
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

    if (mode == 0) {
        cmd = MTG();
    } else {
        cmd = BF();
    }
    return cmd;
}

// ----------- ALGORITHMS --------------------

geometry_msgs::Twist MTG() {
#ifdef DEBUG
    ROS_INFO("MTG");
#endif
  /*
   * Moves to goal until obstacle encountered or goal reached.
   *
   */

  bool isCollision = detectCollision(); // Detect collision.

  // Robot encounters an object.
  // Change mode to BF and save initial collision point.
  if (isCollision) {
    mode = 1;
    BF_init_pose.set(curr_pose);

    return move_cmd(0,0); // No move.
  }
  // No obstacle encountered.
  // Align and move robot along M line.
  else 
    return align();
}

geometry_msgs::Twist BF() {
#ifdef DEBUG
    ROS_INFO("BF");
#endif 
  /*
   * Follows boundary until boundary circumnavigated or M line found.
   *
   */
  
  // Robot has started BF around the obstacle.
  if (BF_began) {

    // Check if robot at start.
    bool isAtStart = curr_pose.vec_distance(BF_init_pose) <= tolerance_pos;
    // Check if MTG is possible. Initialize as true.
    bool MTGpossible = true;
    
    // If robot returns to initial collision point after starting BF,
    // exit with failure.
    if (isAtStart) {
      ROS_INFO("FAILURE: AT START.");
      return move_cmd(0, 0);
    }

    // If robot finds M line without an obstacle,
    // switch to MTG if possible.
    else if (detectOnMline() && MTGpossible) {
      geometry_msgs::Twist moveAlign = align();
      
      // Turn to face M line.
      if (moveAlign.angular.z > 0) {
        return moveAlign;
      }
      // Already facing M line.
      else {
        // If M line is blocked, continue BF.
        if (detectCollision()) {
          ROS_INFO("M LINE BLOCKED.");
          MTGpossible = false;
          return move_cmd(0, 0);
        }
        // Else, switch to MTG.
        else {
          mode = 0;
          return move_cmd(0, 0);
        }
      }
    }

    // Continue BF.
    else
      return BF_step();
  }
  // Start BF.
  else
    return BF_step();
}

geometry_msgs::Twist BF_step() {
  // 0: Initialize search for closest obstacle.
  if (BF_searchState == 0) {
    startAngle = curr_pose.ang; // Start angle

    minRangeAngle = startAngle;
    minRange = range.at(range_size / 2);

    ROS_INFO("START ANGLE: %f", startAngle);

    // Advance state.
    BF_searchState = 1;
    return move_cmd(0, 0);
  }
  // 1: Start search.
  // 2: Continue search.
  else if (BF_searchState == 1 || BF_searchState == 2) {
    double currRange = range.at(range_size / 2);

    // Update angle of closest object found if necessary.
    if (currRange < minRange) {
      minRangeAngle = curr_pose.ang;
      minRange = currRange;
    }

    ROS_INFO("MIN RANGE ANGLE: %f", minRangeAngle);
    ROS_INFO("CURRENT ANGLE: %f", curr_pose.ang);

    // Perturb from start angle and change state to 2.
    if (BF_searchState == 1) {
      BF_searchState = 2;
      return move_cmd(0, SCAN_VELOCITY);
    }
    else {
      // Advance state once robot has turned almost 2 * PI.

      // Stop at end angle.
      // Account for angle being -PI to PI if necessary.
      if (startAngle > PI - 2 * tolerance_ang) {
        if (curr_pose.ang < startAngle + 2 * tolerance_ang - 2 * PI && 
            curr_pose.ang > startAngle) {

          BF_searchState = 3;
        }
      }
      else {
        if (curr_pose.ang < startAngle + 2 * tolerance_ang &&
            curr_pose.ang > startAngle) {

          BF_searchState = 3;
        }
      }

      // Otherwise keep turning.
      return move_cmd(0, SCAN_VELOCITY);
    }
  }
  // 3: Move tangent to object.
  else {
    ROS_INFO("TANGENT ANGLE: %f", tangentAngle);

    // Find angle that is parallel to the tangent of the object.
    // Account for angle being -PI to PI if necessary.
    if (minRangeAngle > PI / 2)
      tangentAngle = minRangeAngle - 3 * PI / 2;
    else
      tangentAngle = minRangeAngle + PI / 2;

    // 0: Align with angle tangent to object.
    if (BF_moveState == 0) {
      // Robot not aligned with tangent angle.
      if (curr_pose.ang > tangentAngle + tolerance_pos || 
          curr_pose.ang < tangentAngle - tolerance_pos)

        return move_cmd(0, tangentAngle - curr_pose.ang);
      // Robot aligned with tangent angle. Move forward.
      else {
        // Reset states.
        BF_searchState = 0;
        BF_moveState = 0;

        ROS_INFO("--MOVED--");

        // Move forward.
        return move_cmd(2 * step, 0);
      }
    }
  }
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


