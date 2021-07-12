/**
 *
 * @file odom_pub.cpp
 * @brief
 *
 * @code{.unparsed}
 *      _____
 *     /  /::\       ___           ___
 *    /  /:/\:\     /  /\         /  /\
 *   /  /:/  \:\   /  /:/        /  /:/
 *  /__/:/ \__\:| /__/::\       /  /:/
 *  \  \:\ /  /:/ \__\/\:\__   /  /::\
 *   \  \:\  /:/     \  \:\/\ /__/:/\:\
 *    \  \:\/:/       \__\::/ \__\/  \:\
 *     \  \::/        /__/:/       \  \:\
 *      \__\/         \__\/         \__\/
 * @endcode
 *
 * @author sunfu.chou (sunfu.chou@gmail.com)
 * @version 0.1
 * @date 2021-07-12
 *
 */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
ros::Publisher pub;
void odomCallback(nav_msgs::Odometry::ConstPtr odom_ptr)
{
  nav_msgs::OdometryPtr odom(new nav_msgs::Odometry);
  odom->child_frame_id = odom_ptr->child_frame_id;
  odom->header = odom_ptr->header;
  odom->pose = odom_ptr->pose;
  odom->twist = odom_ptr->twist;
  // clang-format off
  odom->twist.covariance = {0.3, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.3};
  // clang-format on
  pub.publish(odom);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_pub");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>("raw_odom", 10, &odomCallback);
  pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  ros::spin();
}
