#include <custom_msgs/Door.h>
#include <custom_msgs/Obstacles.h>
#include <ros/ros.h>
#include <vwalls/DoorConverter.h> 
#include <sstream>

using namespace std

DoorConvertor::DoorConverter(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
    n_ = nh;
    nh_private_ = nh_private;
//Topic1 you want to subscribe  
    sub_ = n_.subscribe("/Door_Segments", 5, &DoorConverter::Doorcallback, this); 
    walls_publisher_ = n_.advertise<custom_msg::Obstacls>("/Vobstacles",5);
    initParams();
}

DoorConverter::~DoorConverter()
{
  ROS_INFO("Destroying findcenter");
}

void DoorConverter::initParams()
{
  // load parameters in launch file
/*
	if(!nh_private_.getParam("/findcenterup/doormaxlen",maxdoorlen))
    {
		maxdoorlen = 1.2;
        std::cout<< "getParam Fail" <<std::endl;
    }
    
    if(!nh_private_.getParam("/findcenterup/doorminlen",mindoorlen))
    {
        mindoorlen = 0.8;
        std::cout<< "getParam Fail" <<std::endl;
    }
    if(!nh_private_.getParam("/findcenterup/refercncey",referencey))
        referencey= 11.5;

*/
}

void DoorConverter::Doorcallback(const custom_msgs::Door& door)
{
   /*custom_msg::Obstacls doorline; //墙位置
   
   
   
   double roll, pitch, yaw;
   geometry_msgs::Twist cmd;
   float dist;//每次订阅位姿并执行回调函数时，判断当前位姿与当前贝塞尔路径点的距离，是否到达
   float angle;//每次订阅位姿估计执行回调函数时，判断当前位姿与当前贝塞尔路径点的角度偏差，是否到达
   int b=0;//贝塞尔下发速度计数
   tf::Quaternion quat;
   tf::quaternionMsgToTF(msg2.pose.pose.orientation, quat);  
   tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
   this->posex=msg2.pose.pose.position.x;
   this->posey=msg2.pose.pose.position.y;
   this->posez=yaw;
   this->poseflag=1;
   this->previousposex=this->currentposex;
   this->currentposex=this->posex;
   if(this->currentposex>this->previousposex)
     {
       this->direction=1;//to lift direction
     }
    else
     {
       this->direction=0;//door passing
     }
   

   */
      custom_msg::Obstacls ob;
      ob[0][0]=ob[0][1]=ob[0][2]=0.0;
      walls_publisher_.publish(ob);   
}

int main(int argc, char **argv) 
{ 
//Initiate ROS  
 ros::init(argc, argv, "doorcv"); 
//Create an object of class SubscribeAndPublish that will take care of everything 
 ros::NodeHandle nh;
 ros::NodeHandle nh_private("~");
  
 DoorConverter test(nh, nh_private); 
//ros::spin(); 
 ros::MultiThreadedSpinner s(3); //多线程 
 s.spin(); 
 return 0; 
} 












