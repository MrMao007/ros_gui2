#include "simple_navigation_goals/send_goal.h"

void subCallback(scan_map_icp::t_d_m msg)
{
  angle_1 = msg.angle_dist_1; //机器人本体坐标系与世界坐标系的夹角
  std::cout << "line(6):callback function(): angle_1=" << angle_1 << std::endl;
  if(only)
  {
    only = false;
    linear_dist = sqrt(msg.linear_dist_x * msg.linear_dist_x + msg.linear_dist_y * msg.linear_dist_y);
    linear_dist_x = sqrt(msg.linear_dist_x * msg.linear_dist_x);
    linear_dist_y = sqrt(msg.linear_dist_y * msg.linear_dist_y);
    angle_2 = atan2(msg.linear_dist_y, msg.linear_dist_x); //当前点与目标点之间的夹角,分0-90度和90-180度置标志位
    std::cout << "line(14):atan2(y/x) = " << angle_2 << std::endl;
  }

  std::cout << "line(17):机器人本体坐标系与世界坐标系之间的夹角anlge_1=" << angle_1 * 180 / 3.1415926 << std::endl;
  ROS_INFO("line(18):linear_dist_x: [%f]\n", linear_dist_x);
  // ROS_INFO("angle_dist: [%f]\n", msg.angle_dist_2);
} 

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan_map_icp_node/error", 1, subCallback);
  ros::Publisher pub= n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  geometry_msgs::Twist twist;
  geometry_msgs::Vector3 linear;
  geometry_msgs::Vector3 angular;
  flag = 1;
  only = true;
  ros::Rate loop_rate(1);
  loop_rate.sleep();
  while(ros::ok())
  {
    ros::spinOnce();
    std::cout << "line(37):linear_dist_x = " << linear_dist_x << std::endl;
    std::cout << "line(38):linear_dist_y = " << linear_dist_y << std::endl;
    std::cout << "line(39):flag = " << flag << ": step" << flag << "." << std::endl;
    /**
      轮椅驱动代码
      参数意义:　angle_1　机器人本体坐标系与世界坐标系夹角
      1.根据angle_2(当前点与目标点的角度误差)的大小判断进行顺时针(0-90时)还是逆时针(90-180时)调整角度，使机器人调整到0度或180度角
      2.运动linear_dist_x段距离，与目标点对齐
      3.根据angle_2(当前点与目标点的角度误差)的大小判断进行逆时针(90-180时)还是顺时针(0-90时)调整角度,使机器人调整到90度
      4.运动linear_dist_y段距离，到达目标点
    **/
    if((angle_2>=0)&&(angle_2<=90))
      turn = 1; //turn=1 表示当前位置在参考轴线左侧
    else
      turn = 0; //turn=0　表示当前位置在参考轴线右侧
    
    if(flag==5)
    {
      ROS_INFO("Reach Goal.");
      linear.x = 0;
      linear.y = 0;
      linear.z = 0; 
      angular.x = 0;
      angular.y = 0;
      angular.z = 0;
      twist.linear=linear;
      twist.angular=angular;
      pub.publish(twist);
    }
    
    if(flag==4)
    {
      ROS_INFO("state4");
      linear.x = linear_dist_y;
      linear.y = 0;
      linear.z = 0; 
      angular.x = 0;
      angular.y = 0;
      angular.z = 0;
      twist.linear=linear;
      twist.angular=angular;
      std::cout << "line(68):linear_dist_y = " << linear_dist_y << std::endl;
      if(fabs(linear_dist_y) <= 0.05)
        flag = 5;
      pub.publish(twist);
      loop_rate.sleep();
      //餐桌linear_dist_y = 0.4*linear_dist_y;
      linear_dist_y = 0.5*linear_dist_y;
    }

    if(flag==3)
    {
      ROS_INFO("state3");
      linear.x = 0;
      linear.y = 0;
      linear.z = 0;
      angular.x = 0;
      angular.y = 0;
      /**餐桌if(turn)
        angular.z = fabs(1.570796-angle_1) * 0.3;
      if(!turn)
        angular.z = -fabs(1.570796-angle_1) * 0.3;**/
      /*门*/
      if(turn)
        angular.z = fabs(angle_1) * 0.3;
      if(!turn)
        angular.z = -fabs(angle_1) * 0.3;
      twist.linear=linear;
      twist.angular=angular;
      std::cout << "line(89):angle_1 = " << angle_1 << std::endl;
      /**餐桌if((angle_1 <= 1.67)&&(angle_1 >= 1.47))**/
      /*门*/if(fabs(angle_1 <= 0.1))
        flag =4;
      pub.publish(twist);
    }

    if(flag==2)
    {
      ROS_INFO("state2");
      linear.x = linear_dist_x;
      linear.y = 0;
      linear.z = 0; 
      angular.x = 0;
      angular.y = 0;
      angular.z = 0;
      twist.linear=linear;
      twist.angular=angular;
      std::cout << "line(106):linear_dist_x = " << linear_dist_x << std::endl;
      if(fabs(linear_dist_x) <= 0.05)
        flag = 3;
      pub.publish(twist);
      loop_rate.sleep();
      // 餐桌linear_dist_x = 0.5*linear_dist_x ;
      linear_dist_x = 0.35*linear_dist_x ;
    }

    if(flag==1)
    {
      std::cout << "line(115):state1: angle_1 = " << angle_1 << std::endl;
      ROS_INFO("state1");
      linear.x = 0;
      linear.y = 0;
      linear.z = 0;
      angular.x = 0;
      angular.y = 0;
      /**餐桌if(turn)
        angular.z = -angle_1 * 0.3;
      if(!turn)
        angular.z = +(3.1415926-angle_1) * 0.3;**/
      if(turn)
        angular.z = +(angle_1 +1.5707) * 0.3;
      if(!turn)
        angular.z = -(angle_1 +1.5707) * 0.3;
      twist.linear=linear;
      twist.angular=angular;
      /**餐桌if(fabs(angle_1) <= 0.1 || fabs(3.1415926-angle_1) <= 0.1)**/
      /*门*/if(fabs(angle_1 +1.5707) <= 0.1)
        flag =2;
      pub.publish(twist);
    }
    
  }
  return 0;
}
