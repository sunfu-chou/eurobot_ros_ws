#include "ros/ros.h"
#include <iostream>
#include <std_msgs/Int32MultiArray.h>
#include "vector"

using namespace std;
int cup_count=1;

int main(int argc,char **argv){
    ros::init(argc, argv, "cup_state");
    ros::NodeHandle n;

    ros::Publisher cup_pub = n.advertise<std_msgs::Int32MultiArray>("cup_state",100);
    std_msgs::Int32MultiArray cup_info;
    ros::Rate rate(10);
    ros::Time begin_time = ros::Time::now();

    while ( ros::ok() ){
        ros::Time current = ros::Time::now();
        if( ( current.toSec() - begin_time.toSec() ) >=10.0 && cup_count<=24){
            cup_info.data.push_back(24-cup_count);
            cup_count += 1;
            ROS_INFO("left number : %d",cup_count);
            begin_time = current;
        }
        cup_pub.publish( cup_info );
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}