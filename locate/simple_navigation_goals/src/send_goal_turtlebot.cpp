#include "simple_navigation_goals/send_goal.h"

void subCallback(scan_map_icp::t_d_m msg)
{
  linear_dist = sqrt(msg.linear_dist_x * msg.linear_dist_x + msg.linear_dist_y * msg.linear_dist_y);
  angle_1 = msg.angle_dist_1;
  angle_error = msg.angle_dist_2;
  angle_2 = atan2(msg.linear_dist_y, msg.linear_dist_x);
  ROS_INFO("callback function: angle = [%f], linear_dist = [%f]", double(angle_2), double(linear_dist));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/scan_map_icp_node/error", 1, subCallback);
  ros::Publisher pub= n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::Publisher state_pub_ = n.advertise<std_msgs::Int8>("/state", 1);

  geometry_msgs::Twist twist;
  geometry_msgs::Vector3 linear;
  geometry_msgs::Vector3 angular;
  turn = false;
  ros::Rate loop_rate(15);

  while(ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();
    if((linear_dist >= 0.01) && (turn==false))
    {
      linear.x = 0.2 * linear_dist;
      linear.y = 0;
      linear.z = 0; 
      angular.x = 0;
      angular.y = 0;
      angular.z = -0.3 * (angle_1 - angle_2);
      twist.linear=linear;
      twist.angular=angular;
      pub.publish(twist);
      ROS_INFO("state1");
    }
    else
    {
      turn = true;
      linear.x = 0;
      linear.y = 0;
      linear.z = 0; 
      angular.x = 0;
      angular.y = 0;
      angular.z = -0.15 * angle_error;
      twist.linear=linear;
      twist.angular=angular;
      pub.publish(twist);
      ROS_INFO("angle_error: %f", angle_error);
      if(angle_error <= 0.01)
      {
        linear.x = 0;
        linear.y = 0;
        linear.z = 0; 
        angular.x = 0;
        angular.y = 0;
        angular.z = 0;
        twist.linear=linear;
        twist.angular=angular;
        state.data = 1;
        pub.publish(twist);
        state_pub_.publish(state);
        ROS_INFO("STOP.");
      }
    }
  }

  return 0;
}


/*
  linear.x = (k1 * delta_x) / sqrt(1 + power(delta_x,2) + power(delta_y,2));
  angular.z = (k3 * sin(delta_theta)) / sqrt(1 + power(delta_x,2) + power(delta_y,2));

*/