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
#include "../include/ros_gui/mapNode.h"
#define pi 3.14159256

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Ui{

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
/*****************************************************************************
** Implementation
*****************************************************************************/

MapNode::MapNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
    {}

MapNode::~MapNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

bool MapNode::init() {
    ros::init(init_argc,init_argv,"forbid");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    startp_sub = n.subscribe<geometry_msgs::PointStamped>("startp", 10, &MapNode::startpCallback, this);
    endp_sub = n.subscribe<geometry_msgs::PointStamped>("endp", 10, &MapNode::endpCallback, this);

    line_startp_sub = n.subscribe<geometry_msgs::PointStamped>("line_startp", 10, &MapNode::line_startpCallback, this);
    line_endp_sub = n.subscribe<geometry_msgs::PointStamped>("line_endp", 10, &MapNode::line_endpCallback, this);

    pointp_sub = n.subscribe<geometry_msgs::PointStamped>("pointp", 10, &MapNode::pointpCallback, this);

    semanticp_sub = n.subscribe<geometry_msgs::PoseStamped>("semanticp", 10, &MapNode::semanticpCallback, this);
    goal_sub = n.subscribe<geometry_msgs::PoseStamped>("multigoal", 10, &MapNode::goalCallback, this);

    coarse_sub = n.subscribe<geometry_msgs::PoseStamped>("coarse_goal", 10, &MapNode::coarseCallback, this);
    path_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 10, &MapNode::pathCallback, this);

    markerarray_pub = n.advertise<visualization_msgs::MarkerArray>("forbidden_marker", 10);
    semantic_markerarray_pub = n.advertise<visualization_msgs::MarkerArray>("semantic_marker", 10);
    goalmarker_pub = n.advertise<visualization_msgs::MarkerArray>("goalmarker",10);
    goalid_pub = n.advertise<visualization_msgs::MarkerArray>("goalid_marker",10);
    idarray_pub = n.advertise<visualization_msgs::MarkerArray>("id_marker", 10);
    obstacle_pub = n.advertise<custom_msgs::Obstacles>("Vobstacls", 10);
    fine_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    pathmarker_pub = n.advertise<visualization_msgs::MarkerArray>("path_marker", 10);

    start();
    return true;
}



void MapNode::run() {
    while(1);
    //ros::spin();
}


void MapNode::startpCallback(const geometry_msgs::PointStampedConstPtr &sp){
    startp[0] = sp->point.x;
    startp[1] = sp->point.y;
    emit startpUpdated(startp[0], startp[1]);
}

void MapNode::endpCallback(const geometry_msgs::PointStampedConstPtr &ep){
    endp[0] = ep->point.x;
    endp[1] = ep->point.y;
    emit endpUpdated(endp[0], endp[1]);
}

void MapNode::line_startpCallback(const geometry_msgs::PointStampedConstPtr &sp){
    line_startp[0] = sp->point.x;
    line_startp[1] = sp->point.y;
    emit line_startpUpdated(line_startp[0], line_startp[1]);
}

void MapNode::line_endpCallback(const geometry_msgs::PointStampedConstPtr &ep){
    line_endp[0] = ep->point.x;
    line_endp[1] = ep->point.y;
    emit line_endpUpdated(line_endp[0], line_endp[1]);
}

void MapNode::pointpCallback(const geometry_msgs::PointStampedConstPtr &sp){
    pointp[0] = sp->point.x;
    pointp[1] = sp->point.y;
    emit pointpUpdated(pointp[0], pointp[1]);
}

void MapNode::semanticpCallback(const geometry_msgs::PoseStampedConstPtr &sp){
    geometry_msgs::PoseStamped temp=*sp;
    semantic_vec.push_back(temp);
    semanticp[0] = sp->pose.position.x;
    semanticp[1] = sp->pose.position.y;
    emit semanticpUpdated(semanticp[0], semanticp[1]);
}

void MapNode::pathCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &traj){
    if(record_flag == 0)
        return;
    geometry_msgs::Pose temp = traj->pose.pose;
    if(empty_flag == 1){
        last_pose = temp;
        path.push_back(temp);
        add_path_makerarray(temp);
        empty_flag = 0;
    }
    else{
        if((std::pow(temp.position.x-last_pose.position.x, 2) + std::pow(temp.position.x-last_pose.position.x, 2) + std::pow(temp.position.x-last_pose.position.x, 2)) >= 0.25){
            last_pose = temp;
            path.push_back(temp);
            add_path_makerarray(temp);
        }

    }
}

void MapNode::marker_slot(){
    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "/map";
    tmp.header.stamp = ros::Time::now();
    tmp.ns = "66";
    tmp.action = visualization_msgs::Marker::ADD;
    tmp.id = id++;
    tmp.type = visualization_msgs::Marker::CUBE;
    tmp.pose.orientation.z = 1.0;
    tmp.pose.position.x = (startp[0] + endp[0])/2;
    tmp.pose.position.y = (startp[1] + endp[1])/2;
    tmp.pose.position.z = 0;
    tmp.color.r = 1.0;
    tmp.color.a = 0.5;
    tmp.scale.x = std::abs(startp[0]-endp[0]);
    tmp.scale.y = std::abs(startp[1]-endp[1]);
    tmp.scale.z = 0;
    markerarray.markers.push_back(tmp);
    markerarray_pub.publish(markerarray);
    custom_msgs::Form temp_form;
    geometry_msgs::Point tp;
    tp.x = startp[0]; tp.y=startp[1]; tp.z = 0;
    temp_form.form.push_back(tp);
    tp.x = startp[0]; tp.y=endp[1]; tp.z = 0;
    temp_form.form.push_back(tp);
    tp.x = endp[0]; tp.y=endp[1]; tp.z = 0;
    temp_form.form.push_back(tp);
    tp.x = endp[0]; tp.y=startp[1]; tp.z = 0;
    temp_form.form.push_back(tp);
    obstacles.list.push_back(temp_form);
    obstacle_pub.publish(obstacles);

    visualization_msgs::Marker id_tmp;
    id_tmp.header.frame_id = "/map";
    id_tmp.header.stamp = ros::Time::now();
    id_tmp.ns = "id";
    id_tmp.id = id -1;
    id_tmp.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    id_tmp.scale.z = 1;
    id_tmp.color.g = 1;
    id_tmp.color.a = 1;
    id_tmp.pose.orientation.z = 1.0;
    id_tmp.pose.position.x = (startp[0] + endp[0])/2;
    id_tmp.pose.position.y = (startp[1] + endp[1])/2;
    id_tmp.pose.position.z = 0;
    std::ostringstream str;
    str << id;
    id_tmp.text = str.str();
    idarray.markers.push_back(id_tmp);
    idarray_pub.publish(idarray);
}

void MapNode::line_marker_slot(){
    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "/map";
    tmp.header.stamp = ros::Time::now();
    tmp.ns = "66";
    tmp.action = visualization_msgs::Marker::ADD;
    tmp.id = id++;
    tmp.type = visualization_msgs::Marker::LINE_STRIP;
    tmp.color.r = 1.0;
    tmp.color.a = 0.5;
    tmp.scale.x = 0.1;

    custom_msgs::Form temp_form;
    geometry_msgs::Point tp;
    tp.x = line_startp[0]; tp.y=line_startp[1]; tp.z = 0;
    temp_form.form.push_back(tp);
    tmp.points.push_back(tp);
    tp.x = line_endp[0]; tp.y=line_endp[1]; tp.z = 0;
    temp_form.form.push_back(tp);
    tmp.points.push_back(tp);
    markerarray.markers.push_back(tmp);
    markerarray_pub.publish(markerarray);
    obstacles.list.push_back(temp_form);
    obstacle_pub.publish(obstacles);

    visualization_msgs::Marker id_tmp;
    id_tmp.header.frame_id = "/map";
    id_tmp.header.stamp = ros::Time::now();
    id_tmp.ns = "id";
    id_tmp.id = id -1;
    id_tmp.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    id_tmp.scale.z = 1;
    id_tmp.color.g = 1;
    id_tmp.color.a = 1;
    id_tmp.pose.orientation.z = 1;
    id_tmp.pose.position.x = (line_startp[0] + line_endp[0])/2;
    id_tmp.pose.position.y = (line_startp[1] + line_endp[1])/2;
    id_tmp.pose.position.z = 0;
    std::ostringstream str;
    str << id;
    id_tmp.text = str.str();
    idarray.markers.push_back(id_tmp);
    idarray_pub.publish(idarray);
}

void MapNode::point_marker_slot(double radius){
    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "/map";
    tmp.header.stamp = ros::Time::now();
    tmp.ns = "66";
    tmp.action = visualization_msgs::Marker::ADD;
    tmp.id = id++;
    tmp.type = visualization_msgs::Marker::CYLINDER;
    tmp.color.r = 1.0;
    tmp.color.a = 0.5;
    tmp.scale.x = radius;
    tmp.scale.y = radius;
    tmp.pose.orientation.z=1;
    tmp.pose.position.x=pointp[0];
    tmp.pose.position.y=pointp[1];

    custom_msgs::Form temp_form;
    geometry_msgs::Point tp;
    for(int i = 0; i < 8; i++){
        tp.x = pointp[0] + cos(pi*i/4.0)*radius;
        tp.y = pointp[1] + sin(pi*i/4.0)*radius;
        tp.z = 0;
        temp_form.form.push_back(tp);
    }
    //tp.x = pointp[0]; tp.y=pointp[1]; tp.z = 0;
    //temp_form.form.push_back(tp);
    markerarray.markers.push_back(tmp);
    markerarray_pub.publish(markerarray);
    obstacles.list.push_back(temp_form);
    obstacle_pub.publish(obstacles);

    visualization_msgs::Marker id_tmp;
    id_tmp.header.frame_id = "/map";
    id_tmp.header.stamp = ros::Time::now();
    id_tmp.ns = "id";
    id_tmp.id = id -1;
    id_tmp.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    id_tmp.scale.z = 1;
    id_tmp.color.g = 1;
    id_tmp.color.a = 1;
    id_tmp.pose.orientation.z = 1;
    id_tmp.pose.position.x = pointp[0];
    id_tmp.pose.position.y = pointp[1];
    id_tmp.pose.position.z = 0;
    std::ostringstream str;
    str << id;
    id_tmp.text = str.str();
    idarray.markers.push_back(id_tmp);
    idarray_pub.publish(idarray);
}

void MapNode::delete_slot(int id){
    markerarray.markers.at(id-1).color.a = 0;
    idarray.markers.at(id-1).color.a = 0;
    obstacles.list.at(id-1).form.clear();
    idarray_pub.publish(idarray);
    obstacle_pub.publish(obstacles);
    markerarray_pub.publish(markerarray);
}

void MapNode::semantic_slot(std::string text){
    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "/map";
    tmp.header.stamp = ros::Time::now();
    tmp.ns = "semantic";
    tmp.action = visualization_msgs::Marker::ADD;
    tmp.id = semantic_id++;
    tmp.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    tmp.color.g = 1.0;
    tmp.color.r = 1.0;
    tmp.color.a = 1.0;
    tmp.scale.z = 0.5;
    tmp.pose.orientation.z=1;
    tmp.pose.position.x=semanticp[0];
    tmp.pose.position.y=semanticp[1];
    tmp.text = text;

    semantic_markerarray.markers.push_back(tmp);
    semantic_markerarray_pub.publish(semantic_markerarray);

}

void MapNode::goalCallback(const geometry_msgs::PoseStampedConstPtr &goal){
    visualization_msgs::Marker tmp;
    geometry_msgs::PoseStamped temp = coarsetofine(goal);
    goal_vec.push_back(temp);
    tmp.header.frame_id = "/map";
    tmp.header.stamp = ros::Time::now();
    tmp.ns = "goal";
    tmp.action = visualization_msgs::Marker::ADD;
    tmp.id = goal_id;
    tmp.type = visualization_msgs::Marker::ARROW;
    tmp.pose = temp.pose;
    tmp.color.b = 1.0;
    tmp.color.g = 0;
    tmp.color.r = 0;
    tmp.color.a = 1;
    tmp.scale.x = 0.5;
    tmp.scale.y = 0.05;
    tmp.scale.z = 0.05;

    goal_markerarray.markers.push_back(tmp);
    goalmarker_pub.publish(goal_markerarray);

    visualization_msgs::Marker text_tmp;
    text_tmp.header.frame_id = "/map";
    text_tmp.header.stamp = ros::Time::now();
    text_tmp.ns = "goal_id";
    text_tmp.action = visualization_msgs::Marker::ADD;
    text_tmp.id = goal_id;
    text_tmp.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_tmp.color.r = 1.0;
    text_tmp.color.b = 1.0;
    text_tmp.color.a = 1.0;
    text_tmp.scale.z = 0.5;
    text_tmp.pose=temp.pose;
    text_tmp.text = std::to_string(goal_id+1);

    goalid_markerarray.markers.push_back(text_tmp);
    goalid_pub.publish(goalid_markerarray);

    goal_id++;
}

void MapNode::coarseCallback(const geometry_msgs::PoseStampedConstPtr &goal){
    geometry_msgs::PoseStamped fine = coarsetofine(goal);
    fine_pub.publish(fine);
}

geometry_msgs::PoseStamped MapNode::coarsetofine(const geometry_msgs::PoseStampedConstPtr &goal){
    geometry_msgs::PoseStamped res;
    res.header = goal->header;
    res.pose = goal->pose;
    for(int i = 0 ; i < semantic_vec.size(); i++){
        if((std::pow(semantic_vec[i].pose.position.x - goal->pose.position.x,2) + std::pow(semantic_vec[i].pose.position.y - goal->pose.position.y,2)) <= 4){
            res.pose = semantic_vec[i].pose;
            return res;
        }
    }
    return res;
}




void MapNode::send_multigoal(){
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

void MapNode::multigoal_slot(){
    QtConcurrent::run(this,&MapNode::send_multigoal);
}

void MapNode::record_path_slot(){
    path_id = 0;
    std::vector<geometry_msgs::Pose>().swap(path);
    clear_path_markerarray();
    empty_flag = 1;
    record_flag = 1;
}

void MapNode::save_path_slot(){
    record_flag = 0;
    QString data = "";
    QFile file("/home/mty/bash/path.dat");
    if(file.open(QIODevice::ReadWrite | QIODevice::Text)){
        QTextStream stream(&file);
        for(int i = 0; i < path.size(); i++){
            stream << path[i].position.x << " " << path[i].position.y << " " << path[i].position.z << " " << path[i].orientation.x << " " << path[i].orientation.y << " " << path[i].orientation.z << " " << path[i].orientation.w << "\n";
        }
        file.close();
    }
}

void MapNode::track_slot(){
    track_flag = 1;
}

void MapNode::track_shut_slot(){
    track_flag = 0;
}

void MapNode::add_path_makerarray(geometry_msgs::Pose last){
    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "/map";
    tmp.header.stamp = ros::Time::now();
    tmp.ns = "path";
    tmp.action = visualization_msgs::Marker::ADD;
    tmp.id = path_id++;
    tmp.type = visualization_msgs::Marker::CYLINDER;
    tmp.color.b = 1.0;
    tmp.color.r = 1.0;
    tmp.color.a = 1.0;
    tmp.scale.x = 0.1;
    tmp.scale.y = 0.1;
    tmp.pose.position.x = last.position.x;
    tmp.pose.position.y = last.position.y;

    path_markerarray.markers.push_back(tmp);
    pathmarker_pub.publish(path_markerarray);
}

void MapNode::clear_path_markerarray(){
    std::vector<visualization_msgs::Marker>().swap(path_markerarray.markers);
    pathmarker_pub.publish(path_markerarray);
}
}  // namespace ros_gui

