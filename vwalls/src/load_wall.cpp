#include <ros/ros.h>
#include <custom_msgs/Door.h>

void msgCallback(const custom_msgs::Door::ConstPtr &msg)
{
  //test_msgs::Test类型里的float32[]数据传到vector
  float array[2] = msg->leftside;

  std::cout << "leftside=" << array[0] << std::endl;
}

int main(int argc,char ** argv)
{
  ros::init(argc,argv,"test1");
  ros::NodeHandle n;
  ros::Subscriber msg_sub = n.subscribe("/door", 100, msgCallback);
  ros::spin();
  return 0;
}