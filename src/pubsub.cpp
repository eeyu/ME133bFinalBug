#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>

static double cumulative = 0;

geometry_msgs::Twist get_move() {
    geometry_msgs::Twist cmd;
    double forward_cmd = 1.0;
    double turn_cmd = 1.0;
    cmd.linear.x = forward_cmd;
    cmd.angular.z = turn_cmd;
    return cmd;
}


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  cumulative += msg->ranges[0];
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Publisher command_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000); // TODO publish correct message type
  ros::Rate loop_rate(10);

  ros::Subscriber subLaser = n.subscribe<sensor_msgs::LaserScan>("scan", 1000, scanCallback);
  //ros::Subscriber subPos = n.subscribe<nav_msgs::Odometry>("odom", 1000, posCallback);

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    geometry_msgs::Twist cmd;

    cmd = get_move();

    command_pub.publish(cmd);

    ROS_INFO("%f", cmd.linear.x);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
