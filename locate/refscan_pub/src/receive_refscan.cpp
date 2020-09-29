#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>

bool receiveFlag = false;
const int num_readings = 720;
const int laser_frequency = 40;
double ref_ranges[10][num_readings] ={0.0};
double ref_intensities[10][num_readings] = {0.0};
// string frame_id;
double  angle_min, angle_max, angle_increment, range_min, range_max;
int count = 0;
// @brief: 读取10帧激光数据并存入参考点云
// @param: void
// @ret: 返回参考点云
// @birth: 2020.09.23
void SubCallback(sensor_msgs::LaserScan msg)
{
	// frame_id= msg.header.frame_id;
	angle_min = msg.angle_min;
	angle_max = msg.angle_max;
	angle_increment = msg.angle_increment;
	range_min = msg.range_min;
	range_max = msg.range_max;
	for (int j = 0; j < num_readings; ++j)
		ref_ranges[count][j] = msg.ranges[j];
	count++;
	if (count == 10)
		receiveFlag = true;
	std::cout << "callback function() count = " << count << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "receiveRefscan");
	ros::NodeHandle nh;

	ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, SubCallback);
	ros::Publisher laserPub = nh.advertise<sensor_msgs::LaserScan>("/refscan", 1);

	// @func:: 定义局部变量
	// @param: reflaser--sensor_msgs::LaserScan类对象
	sensor_msgs::LaserScan reflaser;
	reflaser.ranges.resize(num_readings);
	reflaser.intensities.resize(num_readings);
	while (count < 10)
		ros::spinOnce();
	if (receiveFlag == true)
	{
		for(int i = 0; i < 10; i++)
		{
	    	for(int j = 0; j < num_readings; j++)
		    {
		    	reflaser.ranges[j] += ref_ranges[i][j] * 0.1;
		    	reflaser.intensities[j] = ref_intensities[i][j];
		    }
	    }
	
	    ros::Rate r(100.0);
		while(nh.ok())
		{
			ros::Time scan_time = ros::Time::now();
			//　桌子的参考激光数据
		    reflaser.header.frame_id = "/2dlaser1_link";
		    reflaser.angle_min = angle_min;
		    reflaser.angle_max = angle_max;
		    reflaser.angle_increment = angle_increment;
		    reflaser.time_increment = 0;
		    reflaser.range_min = range_min;
		    reflaser.range_max = range_max;
		    reflaser.header.stamp = scan_time;
		    
		    laserPub.publish(reflaser);
			ROS_INFO("Table");
		    r.sleep();
		}
	}
	return 0;
}