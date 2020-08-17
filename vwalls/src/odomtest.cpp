#include <cmath>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;
double x, y, z, ww, zz, hh, ii, Aww, Azz, Ahh, Aii;
double theta;
int main(int argc, char **argv) {
  ros::init(argc, argv, "getpose");

  ros::NodeHandle node;
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  while (node.ok()) {
    tf::StampedTransform transform;
    try {
      //得到坐标odom和坐标base_link之间的关系
      listener.waitForTransform("odom", "base_link", ros::Time(0),
                                ros::Duration(3.0));
      listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }


    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "base_link";
    // we'll just use the most recent transform available for our simple example
    laser_point.header.stamp = ros::Time(0);
    // just an arbitrary point in space
    laser_point.point.x = 0.0;
    laser_point.point.y = 0.0;
    laser_point.point.z = 0.0;

    try {

      geometry_msgs::PointStamped base_point;
      listener.transformPoint("map", laser_point, base_point);
      ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, "
               "%.2f) at time %.2f",
               laser_point.point.x, laser_point.point.y, laser_point.point.z,
               base_point.point.x, base_point.point.y, base_point.point.z,
               base_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
    }
    
/*
       x=transform.getOrigin().x();
       y=transform.getOrigin().y();
       z=transform.getOrigin().z();
       cout<<"x: "<<x<<endl;
       cout<<"y: "<<y<<endl;
       cout<<"z: "<<z<<endl;
       //两种不同的表示方法，来表示getRotation
        ww=transform.getRotation()[0];
        zz=transform.getRotation()[1];
        hh=transform.getRotation()[2];
        ii=transform.getRotation()[3];
        cout<<"ww: "<<ww<<endl;
        cout<<"zz: "<<zz<<endl;
        cout<<"hh: "<<hh<<endl;
        cout<<"ii: "<<ii<<endl;
        Aww=transform.getRotation().getX();
        Azz=transform.getRotation().getY();
        Ahh=transform.getRotation().getZ();
        Aii=transform.getRotation().getW();
        theta=2*acos(transform.getRotation().getW())/3.1415926*180;
        cout<<"Aww: "<<Aww<<endl;
        cout<<"Azz: "<<Azz<<endl;
        cout<<"Ahh: "<<Ahh<<endl;
        cout<<"Aii: "<<Aii<<endl;
        cout<<"Theta: "<<theta<<endl;
        cout<<endl;
*/    
    rate.sleep();
  }
  return 0;
};
