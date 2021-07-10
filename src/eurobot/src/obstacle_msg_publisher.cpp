/**
 *
 * @file obstacle_msg.cpp
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
 * @date 2021-07-09
 *
 */

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

costmap_converter::ObstacleMsg xy_to_obstacle_msg(double x, double y)
{
  costmap_converter::ObstacleMsg obstacle_msg;
  obstacle_msg.orientation.w = 1.0;
  geometry_msgs::Point32 point;
  point.x = x + 0.1;
  point.y = y + 0.1;
  obstacle_msg.polygon.points.push_back(point);
  point.x = x - 0.1;
  point.y = y + 0.1;
  obstacle_msg.polygon.points.push_back(point);
  point.x = x - 0.1;
  point.y = y - 0.1;
  obstacle_msg.polygon.points.push_back(point);
  point.x = x + 0.1;
  point.y = y - 0.1;
  obstacle_msg.polygon.points.push_back(point);

  return obstacle_msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_msg_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("/move_base/TebLocalPlannerROS/obstacles", 10);
  ros::Rate rate(1);

  costmap_converter::ObstacleArrayMsg obstacles_array_;

  double x;
  double y;

  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    obstacles_array_.header.frame_id = "map";
    obstacles_array_.header.stamp = now;
    obstacles_array_.obstacles.clear();
    obstacles_array_.obstacles.push_back(xy_to_obstacle_msg(1.0, 1.5));

    pub.publish(obstacles_array_);

    ROS_INFO_STREAM(obstacles_array_);
    rate.sleep();
  }
}
