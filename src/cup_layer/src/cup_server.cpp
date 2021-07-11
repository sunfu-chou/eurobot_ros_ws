#include "ros/ros.h"
#include <iostream>
#include "cup_layer/cup_state.h"
#include "vector"

using namespace std;

vector<int> cup_num= { 24 };
int cup = 24;
ros::Time begin_time;
int count_time = 0;

bool cup_detect(cup_layer::cup_state::Request &req, cup_layer::cup_state::Response &res){
    ros::Time call = ros::Time::now();
    if( ( call.toSec() - begin_time.toSec() ) >= 10.0 ){
        begin_time = call;
        cup = cup-1;
        cup_num.push_back(cup);
    }
    else{
    }
    res.delete_cup = cup_num;
    return true;
}

int main(int argc,char **argv){
    ros::init(argc, argv, "cup_state");
    ros::NodeHandle n;

    ros::ServiceServer cup_server = n.advertiseService("cup_state",cup_detect);
    if( count_time == 0){
        begin_time = ros::Time::now();
    }
    ros::spin();
    return 0;
}