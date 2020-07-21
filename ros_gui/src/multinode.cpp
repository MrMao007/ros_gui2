/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include "../include/ros_gui/multinode.h"
#include <std_msgs/Float32.h>
#include <QtConcurrent/QtConcurrent>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Ui {

/*****************************************************************************
** Implementation
*****************************************************************************/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Multinode::Multinode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

Multinode::~Multinode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool Multinode::init() {
    ros::init(init_argc,init_argv,"multinode");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    goal_sub = n.subscribe<geometry_msgs::PoseStamped>("multigoal", 100, &Multinode::callback, this);
    //std::cout << "ok123" << std::endl;
    start();
	return true;
}



void Multinode::run() {
    while(1);
}


void Multinode::callback(const geometry_msgs::PoseStampedConstPtr &goal_msg){
    geometry_msgs::PoseStamped tmp;
    tmp.header = goal_msg->header;
    tmp.pose = goal_msg->pose;
    goal_vec.push_back(tmp);
    //
}

void Multinode::send_multigoal(){
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal tmp;
    for(int i  = 0; i<goal_vec.size(); i++){
        tmp.target_pose.header = goal_vec[i].header;
        tmp.target_pose.pose = goal_vec[i].pose;
        ac.sendGoal(tmp);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Goal %d succeeded", i+1);
        }
        else{
            ROS_INFO("FAIL!");
            break;
        }
    }
}

void Multinode::multigoal_slot(){
    QtConcurrent::run(this,&Multinode::send_multigoal);
}

}  // namespace ros_gui

