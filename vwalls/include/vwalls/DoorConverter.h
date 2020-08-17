#ifndef DOORCONVERTER_H
#define DOORCONVERTER_H

#include <custom_msgs/Door.h>
#include <custom_msgs/Obstacles.h>
#include <ros/ros.h>
#include <sstream>

class DoorConverter
{
    public:
        DoorConverter();
        ~DoorConverter();

    private:
        ros::NodeHandle n_; 
        ros::NodeHandle nh_private_;
        ros::Publisher walls_publisher_;
        ros::Subscriber sub_;

    private://member functions
        void initParams();
        void Doorcallback(const custom_msgs::Door& door); 

}

#endif