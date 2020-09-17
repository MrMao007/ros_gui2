#include "scan_map_icp/scan_map_icp.h"
tf::StampedTransform msg_refpose;

int main(int argc, char **argv)
{
  // nodename = scan_map_icp_node
  ros::init(argc, argv, "scan_map_icp_node");
  //geometry_msgs::PoseWithCovarianceStamped msg_poseref;
  // msg_refpose.frame_id_ = "map";
  // msg_refpose.child_frame_id_ = "base_link";
  // msg_refpose.stamp_ = ros::Time(0);
  // msg_refpose.setOrigin(tf::Vector3(6.2453, -2.8831, 0));
  // tf::Quaternion q;
  // q.setRPY(0, 0, 1.19409);
  // msg_refpose.setRotation(q);
  scan_map_icp::ScanMapIcp icp; //变量icp包含三个参数:inlier_percent, dist, angle_dist
  
  
  /******/


  /****************
  **
	将读到的参考点云信息转化到二维平面坐标系下
  **
  ****************/
  /***
  void reflaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
  {
  	std::vector<float> ranges = msg->ranges;
  	for(int i = 0; i < ranges; i++)
  	{
  		double angle = msg.angle_min + i * msg.angle_increment;
  		double X = ranges[i] * cos(angle) + originX;
  		double Y = ranges[i] * sin(angle) + originY;
  		float intensity = msg.intensities[i];
  		std::cout << ranges[i] << ", " << endl;
  	}	
  }
  int main(int argc, char **argv)
  {
  	ros::init(argc, argv, "reflaser_listener");
  	ros::NodeHandle nh;

  	ros::Subscriber sub = nh.subscribe("/refscan", 10, reflaserCallback);

  	ros::spin;

  	return 0;

  }
  ***/

  /****************
  **
	录入参考点机器人的位姿
  **
  ****************/
  // version 1
  /***
  geometry_msgs::PoseWithCovarianceStamped msg_poseref;
  msg_poseref.header.frame_id = "map";
  msg_poseref.header.stamp = ros::Time::now();
  msg_poseref.pose.pose.position.x = 6.2453;
  msg_poseref.pose.pose.position.y = -2.8831;
  msg_poseref.pose.pose.position.z = 0;
  msg_poseref.pose.pose.orientation.x = 0.0;
  msg_poseref.pose.pose.orientation.y = 0.0;
  msg_poseref.pose.pose.orientation.z = -0.5622;
  msg_poseref.pose.pose.orientation.w = 0.8270;


  // version 2  
  static tf::TransformBroadcaster ref_br;
  // 根据参考点机器人的位置和姿态，得到其相对于世界坐标系的变换
  tf::Trasform transform_RefToWorld;
  transform_RefToWorld.setOrigin(tf::Vector(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, theta);
  transform_RefToWorld.setRotation(q);

  //tf广播器发布机器人参考位姿相对于世界坐标系的位置变换
  ref_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot_ref");
  ***/


  /****************
  **
	读取当前机器人位姿与参考位姿之间的tf
  **
  ****************/
  //定义速度控制指令的消息发布者
  /***
  ros::Publisher robot_vel = n.advertise<geometry_msgs::Twist>("robot/cmd_vel", 10);
  //定义一个tf监听器
  tf::TranformListener listener;

  tf::StampedTransform transform;
  try
  {
  	//监听参考位姿与当前位姿的坐标变换
  	listener.waitForTransform("robot_now", "robot_ref", ros::Time(0), ros::Duration(3.0));
  	listener.lookupTransform("robot_now", "robot_ref", ros::Time(0), transform);
  }
  catch(tf::TransformException &ex)
  {
  	ROS_ERROR("%s", ex.what());
  	ros::Duration(1.0).sleep();
  	continue;
  }
  //根据坐标变换计算得出当前机器人的角速度和线速度，并发布该消息
  geometry_msgs::Twist vel_msgs;
  vel_msgs.angular.z = 4.0 * atan2(transform.getOrigin().y(), transform.getOrigin().x());
  vel_msgs.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(),2) + pow(transform.getOrigin(),y(), 2));

  robot_vel.publish(vel_msgs);
  ***/

  return 0;
}
