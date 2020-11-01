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
#include <string>
#include <sstream>
#include <math.h>
#include "../include/ros_gui/markerNode.h"
#define pi 3.14159256

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Ui {

/*****************************************************************************
** Implementation
*****************************************************************************/

MarkerNode::MarkerNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
    {}

MarkerNode::~MarkerNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

bool MarkerNode::init() {
    ros::init(init_argc,init_argv,"marker");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    twist_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, &MarkerNode::callback, this);
    lmarker_pub = n.advertise<visualization_msgs::Marker>("lmarker", 10);
    amarker_pub = n.advertise<visualization_msgs::MarkerArray>("amarker", 10);
    text_pub = n.advertise<visualization_msgs::Marker>("tmarker", 10);
    power_sub=n.subscribe<std_msgs::Float32>("power",10,&MarkerNode::powerCallback, this);
    power_flag_sub=n.subscribe<std_msgs::Float32>("power_flag",10,&MarkerNode::flagCallback, this);
    temp_sub=n.subscribe<std_msgs::Float32>("temp",10,&MarkerNode::tempCallback, this);

    tmarker.header.frame_id = "/base_link";
    tmarker.header.stamp = ros::Time::now();
    tmarker.ns = "vel";
    tmarker.action = visualization_msgs::Marker::ADD;
    tmarker.id = 4;
    tmarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    tmarker.scale.z = 0.4;
    tmarker.color.b = 255;
    tmarker.color.g = 255;
    tmarker.color.r = 255;
    tmarker.color.a = 0.8;
    tmarker.pose.orientation.w=cos(pi/4);
    tmarker.pose.orientation.y=sin(pi/4);
    tmarker.pose.position.x = -1;
    tmarker.pose.position.z = 1.5;


    amarker.header.frame_id = "/base_link";
    amarker.header.stamp = ros::Time::now();
    amarker.ns = "vel";
    amarker.action = visualization_msgs::Marker::ADD;
    amarker.id = 1;
    amarker.type = visualization_msgs::Marker::LINE_STRIP;
    amarker.pose.orientation.w = 1.0;
    amarker.scale.x=0.1;
    amarker.color.b = 1.0;
    amarker.color.a = 1.0;
    geometry_msgs::Point p;
    p.z = 1.5;
    for(int i = 0; i <= 10; i++){
        p.x=0.7*sin(i/10.0*pi);
        p.y=0.7*cos(i/10.0*pi);

        amarker.points.push_back(p);
    }
    markerarray.markers.push_back(amarker);

    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "/base_link";
    tmp.header.stamp = ros::Time::now();
    tmp.ns = "vel";
    tmp.action = visualization_msgs::Marker::ADD;
    tmp.id = 2;
    tmp.type = visualization_msgs::Marker::ARROW;
    tmp.pose.orientation.z = 1.0;
    tmp.pose.position.x = 0.01;
    tmp.pose.position.y = 0.7;
    tmp.pose.position.z = 1.5;
    tmp.color.b = 1.0;
    tmp.color.a = 1.0;
    tmp.scale.x = 0.4;
    tmp.scale.y = 0.1;
    tmp.scale.z = 0.1;
    markerarray.markers.push_back(tmp);


    tmp.pose.position.y = -0.7;
    tmp.id = 3;
    markerarray.markers.push_back(tmp);


    start();
    return true;
}



void MarkerNode::run() {
    while(1);
    //ros::spin();
}


void MarkerNode::callback(const geometry_msgs::TwistConstPtr &cmd_vel){
    //std::cout << "ok" << std::endl;
    double linear_x = cmd_vel->linear.x;
    double linear_y = cmd_vel->linear.y;
    double angular_z = cmd_vel->angular.z;

    lmarker.header.frame_id = "/base_link";
    lmarker.header.stamp = ros::Time::now();
    lmarker.ns = "vel";
    lmarker.action = visualization_msgs::Marker::ADD;
    lmarker.id = 0;
    lmarker.type = visualization_msgs::Marker::ARROW;
    /*geometry_msgs::Point p;
    p.x = linear_x;p.y = linear_y;p.z = 1;
    lmarker.points.at(1) = p;*/
    lmarker.pose.position.x = 0;
    lmarker.pose.position.y = 0;
    lmarker.pose.position.z = 1.5;
    lmarker.pose.orientation.x = 0.0;
    lmarker.pose.orientation.y = 0.0;
    if(linear_x >=0){
        lmarker.pose.orientation.z = 0.0;
        lmarker.pose.orientation.w = 1.0;
    }
    else{
        lmarker.pose.orientation.z = 1.0;
        lmarker.pose.orientation.w = 0.0;
    }

    lmarker.color.b = 0;
    lmarker.color.g = 1.0;
    lmarker.color.r = 0;
    lmarker.color.a = 1;
    lmarker.scale.x = 4*sqrt(linear_x*linear_x + linear_y*linear_y);
    lmarker.scale.y = 0.4*sqrt(linear_x*linear_x + linear_y*linear_y);
    lmarker.scale.z = 0.4*sqrt(linear_x*linear_x + linear_y*linear_y);
    lmarker_pub.publish(lmarker);

    markerarray.markers[0].header.stamp = ros::Time::now();
    markerarray.markers[1].header.stamp = ros::Time::now();
    markerarray.markers[2].header.stamp = ros::Time::now();
    if(angular_z == 0){
        markerarray.markers[0].color.a = 0;
        markerarray.markers[1].color.a = 0;
        markerarray.markers[2].color.a = 0;
    }
    else if (angular_z > 0){
        markerarray.markers[0].color.a = 1;
        markerarray.markers[1].color.a = 1;
        markerarray.markers[2].color.a = 0;
    }
    else{
        markerarray.markers[0].color.a = 1;
        markerarray.markers[1].color.a = 0;
        markerarray.markers[2].color.a = 1;
    }

    amarker_pub.publish(markerarray);

    tmarker.header.stamp = ros::Time::now();
    std::ostringstream str;
    str << std::setprecision(2) <<"linear: " << sqrt(linear_x*linear_x + linear_y*linear_y) << "m/s\nangular: " << angular_z <<"rad/s";
    tmarker.text=str.str();
    text_pub.publish(tmarker);
}

void MarkerNode::powerCallback(const std_msgs::Float32ConstPtr &p){
    emit power(p->data);
}

void MarkerNode::flagCallback(const std_msgs::Float32ConstPtr &f){
    emit power_flag(f->data);
}

void MarkerNode::tempCallback(const std_msgs::Float32ConstPtr &t){
    emit temp(t->data);
}



}  // namespace ros_gui
