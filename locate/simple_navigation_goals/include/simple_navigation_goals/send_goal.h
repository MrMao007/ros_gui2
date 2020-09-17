#ifndef SEND_GOAL_H_
#define SEND_GOAL_H_

#include <ros/ros.h>
#include <scan_map_icp/t_d_m.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <unistd.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int flag;
std_msgs::Int8 state;
double linear_dist_x;
double linear_dist_y;
double linear_dist;
double angle_1;
double angle_2;
double angle_error;
bool only;
bool turn;
bool run;
ros::Publisher pub;
ros::Publisher state_pub_;

#endif
