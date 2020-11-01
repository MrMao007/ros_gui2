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
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ros_gui/qnode.h"
#include <sensor_msgs/image_encodings.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Ui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"ros_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
    image_transport::ImageTransport it(n);
	// Add your ros communications here.
    joint_sub = n.subscribe<sensor_msgs::JointState>("/my_gen3/joint_states", 1000, &QNode::callback, this);
    image_sub = it.subscribe("/d400/color/image_raw",100,&QNode::img_callback, this);
    //std::cout << "ok123" << std::endl;
    start();
	return true;
}



void QNode::run() {
    while(1);
}


void QNode::callback(const sensor_msgs::JointStateConstPtr &joint_msg){
    //std::cout << "ok" << std::endl;
    joint_states[0] = joint_msg->position[0];
    joint_states[1] = joint_msg->position[1];
    joint_states[2] = joint_msg->position[2];
    joint_states[3] = joint_msg->position[3];
    joint_states[4] = joint_msg->position[4];
    joint_states[5] = joint_msg->position[5];
    joint_states[6] = joint_msg->position[6];
    emit jointUpdated();
}



void QNode::img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        img = cv_ptr->image;
        image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888);//change  to QImage format
        //ROS_INFO("I'm setting picture in mul_t callback function!");
        Q_EMIT loggingCamera();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

}  // namespace ros_gui
