#include "scan_map_icp/scan_map_icp.h"

namespace scan_map_icp
{
// 定义ScanMapIcp类里面的公有成员函数ScanMapIcp()
ScanMapIcp::ScanMapIcp():private_nh_("~") 
/***
* 只有一个话题发布到全局命名空间： topic name = '/call_icp'
* 其余的话题都发布到私有命名空间： '/scan_map_icp_node/' 
***/
{
  /***
  * 1、 初始化ICP匹配用到的参数
  * 2、 从参数服务器读取以下参数：
  *     base_laser_frame--->默认值(hokuyo_link)
  *     odom_frame--------->默认值(odom_combined)
  *     use_sim_time------->默认值(true)
  ***/
  initialParams();  
  // global_nh_.param("base_laser_frame", base_laser_frame_, std::string("/laser"));
  // global_nh_.param("base_laser_frame", base_laser_frame_, std::string("hokuyo_link"));
  global_nh_.param("base_laser_frame", base_laser_frame_, std::string("2dlaser1_link"));
  global_nh_.param("odom_frame", odom_frame_, std::string("/odom"));
  private_nh_.param("use_sim_time", use_sim_time_, true);
  if (use_sim_time_)
  {
    ROS_INFO("use_sim_time true!");
  }
  else
  {
     ROS_INFO("use_sim_time false!");
  }
  last_proccessed_scan_ = ros::Time::now();
  projector_ = new laser_geometry::LaserProjection();
  tf::TransformListener listener;
  listener_ = &listener;
  /*订阅话题 ---> 'scan' | 'map' | 'call_icp' */
  //laser_scan_sub_ = private_nh_.subscribe<sensor_msgs::LaserScan>("/turtlebot/scan", 1, &ScanMapIcp::laserScanSubCallback, this);
  laser_scan_sub_ = private_nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1, &ScanMapIcp::laserScanSubCallback, this);
  map_sub_ = private_nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &ScanMapIcp::mapSubCallback, this);
  call_icp_sub_ = global_nh_.subscribe<std_msgs::String>("/call_icp", 1, &ScanMapIcp::callIcpSubCallback, this);
  /**
  * 订阅参考激光数据 topic:/turtlebot/laser/scan 
  **/
  ref_laser_scan = private_nh_.subscribe<sensor_msgs::LaserScan>("/refscan",10, &ScanMapIcp::ReflaserScanSubCallback, this);// turtlebot和真实轮椅使用的topic
  ref_laser_scan = private_nh_.subscribe<sensor_msgs::LaserScan>("/reftable",10, &ScanMapIcp::ReflaserScanSubCallback, this);
  call_icp_msg_ = std::string("finish");

  /*新发布话题 ---> 'initial_pose' | 'icp_run_info' | 'map_point' | 'scan_point' | 'scan_point_transformed' */
  initial_pose_pub_ = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initial_pose", 1);
  icp_run_info_pub_ = private_nh_.advertise<std_msgs::String>("icp_run_info", 1);
  map_to_cloud_point_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("map_point", 1);
  scan_to_cloud_point_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("scan_point", 1);
  // scan_after_transform_pub_ = private_nh_.advertise<sensor_msgs::PointCloud>("ref_cloud_in_scan", 1);
  scan_after_transform_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("scan_point_transformed", 1);
  source_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("source_pub", 1);
  target_pub = private_nh_.advertise<sensor_msgs::PointCloud2>("target_pub", 1);
  control_cmd = private_nh_.advertise<scan_map_icp::t_d_m>("error", 1);
  dlf_icp_result = global_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
  //transformed_pointcloud = private_nh_.advertise<PointCloudT>("transformed_pointcloud", 1);
  //test = global_nh_.advertise<sensor_msgs::PointCloud2>("test",1);
  // 调试时打开下面的注释
  // laser_point_pub = private_nh_.advertise<sensor_msgs::PointCloud>("laser_scan", 1);
  /***
  *boost::share_ptr 不需要手动释放内存
  *定义三个智能指针:
  * <pcl::PointCloud<pcl::PointXYZ()> >类型的map_cloud_xyz_
  * <sensor_msgs::PointCloud2>类型的map_cloud_ 和 scan_cloud_
  ***/ 
  map_cloud_xyz_ = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());
  map_cloud_ = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
  // 用不到scan_cloud_ = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2());
  listener_->waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(30.0));
  listener_->waitForTransform(base_laser_frame_, "/map", ros::Time(0), ros::Duration(30.0));
  // ros async thread spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();
  updateParams();
  ROS_INFO("ScanMapIcp running");
  
 //调试时开辟一个线程更新参数
 //updateParamsThread_ = new boost::thread(boost::bind(&ScanMapIcp::updateParamsThreadFun,this));
  updateParamsLoop();
}

ScanMapIcp::~ScanMapIcp()
{
// 调试用的时候，析构函数删除线程函数
/*  if(statusLoopThread_)
//  {
//    updateParamsThread_->join();
//    delete updateParamsThread_;
//    statusLoopThread_ = false;
 }*/
}

void ScanMapIcp::initialParams()
{
  icp_fitness_threshold_ = 100;
  dist_threshold_ = 0.05;
  angle_threshold_ = 0.01;
  angle_upper_threshold_ = M_PI / 6;
  age_threshold_ = 1;
  update_age_threshold_ = 1;
  icp_inlier_dist_ = 0.01;
  icp_inlier_threshold_ = 0.9;
  pose_covariance_trans_ = 1.5;
  icp_num_iter_ = 500;
  scan_rate_ = 2;
 // status_loop_thread_ = true;
  last_time_sent_ = -1000;
  act_scan_ = 0;
  last_scan_ = 0;
}

void ScanMapIcp::updateParamsLoop()
{
  ros::Rate loop_rate(5); //设定频率为5Hz，通过睡眠度过一个循环中剩下的时间，已达到设定频率
  while (ros::ok())
  {
    if (act_scan_ > last_scan_)
    {
      last_scan_ = act_scan_;
      if (has_map_)
      {
        map_to_cloud_point_pub_.publish(map_cloud_);
        has_map_ = true;
      }
      if (has_scan_)
      {
        scan_to_cloud_point_pub_.publish(cloud2_);
        has_scan_ = true;
      }
      if (has_scan_transform_)
      {
        scan_after_transform_pub_.publish(cloud2transformed_);
        has_scan_transform_ = false;
      }
    }
    loop_rate.sleep();
    ros::spinOnce();
    if(ros::Time::now() - params_were_updated_ > ros::Duration(1.0))
    {
      updateParams();
    }
  }
}

/**
 * @brief ScanMapIcp::getTree , get cloud search tree
 * @param cloud ,input cloud
 * @return result tree
 */
pcl::KdTree<pcl::PointXYZ>::Ptr ScanMapIcp::getTree(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::KdTree<pcl::PointXYZ>::Ptr tree;
  tree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  tree->setInputCloud(cloud);
  return tree;
}

/**
 * @brief ScanMapIcp::mapSubCallback: map subscribe callback function, convert map to pointcloud
 * @param data
 */
void ScanMapIcp::mapSubCallback(const nav_msgs::OccupancyGridConstPtr &data)
{
  ROS_INFO("I hear frame_id:[%s]", data->header.frame_id.c_str());
  /***
  * 将读入的二维栅格地图中的数据存入到对应的float变量中
  ***/
  float resolution = data->info.resolution;
  float width = data->info.width;
  float height = data->info.height;
  float origin_pose_x = data->info.origin.position.x;
  float origin_pose_y = data->info.origin.position.y;
  
  /***
  * map_cloud_xyz_ 是<pcl::PointCloud<pcl::PointXYZ()> >类型的智能指针
  * 对于有结构点云来说，height代表点云的总行数
  * 对于无结构点云来说，height值是1
  * height的值也经常用来判断点云是不是一个有结构的点云
  ***/ 
  map_cloud_xyz_->height = 1; 
  /***
  * is_dense(bool)
  * 判断map_cloud_xyz_中的数据是否是有限的(有限为true)
  * 或者说是判断点云中的点是否包含inf/NaN这种值(包含为false)
  ***/ 
  map_cloud_xyz_->is_dense = false; 
  std_msgs::Header header;
  header.stamp = ros::Time(0);
  header.frame_id = "/map";
  /***
  * std_msgs::Header类型的数据转化为pcl::PCLHeader类型
  * 也就是说：将数据转化为pcl可以使用的数据类型
  ***/ 
  map_cloud_xyz_->header = pcl_conversions::toPCL(header); 
  /***
  * 定义一个PointXYZ类型的变量point_xyz
  * point_xyz包含三维xyz坐标信息
  ***/
  pcl::PointXYZ point_xyz;
  // ROS_INFO("111111");

  /***
  * fill with pointcloud
  * 根据地图的长和宽，将二维栅格地图转化为点云地图
  * 直接将point_xyz的z坐标设为0
  * 通过point_xyz将数据存入map_cloud_xyz_智能指针
  ***/
  for(int y = 0; y < height; ++y)
  {
    for(int x = 0; x < width; ++x)
    {
      if(data->data[x + y*width] == 100)
      {
        point_xyz.x = (.5f + x) * resolution + origin_pose_x;
        point_xyz.y = (.5f + y) * resolution + origin_pose_y;
        point_xyz.z = 0;
        map_cloud_xyz_->points.push_back(point_xyz);
      }
    }
  }
  /* 对于无组织的点云来说，width代表点云中点的个数; pointcloud's number */ 
  map_cloud_xyz_->width = map_cloud_xyz_->points.size(); 
  ROS_INFO("222222");
  map_tree_ = getTree(map_cloud_xyz_);
  /***
  * map_cloud_xyz_ 为<pcl::PointCloud<pcl::PointXYZ()> >类型
  * map_cloud_ 为<sensor_msgs::PointCloud2>类型
  * 将pcl类型的点云信息转化为sensor_msgs类型的点云信息
  ***/
  pcl::toROSMsg(*map_cloud_xyz_,*map_cloud_);
  
  ROS_INFO("Publishing PointXYZ cloud with %ld points in frame %s", map_cloud_xyz_->points.size(), map_cloud_->header.frame_id.c_str());
  has_map_ = true;
}

/**
 * @brief ScanMapIcp::getTransform, get two frames' translation
 * @param trans, save translation variable
 * @param parent_frame, parent frame
 * @param child_frame, child frame
 * @param stamp, time stamp
 * @return transform state
 * function: 输入一个时间点，给定父坐标系和子坐标系，得到父子坐标系的坐标变换
 */
bool ScanMapIcp::getTransform(tf::StampedTransform &trans, const std::string parent_frame,\
                              const std::string child_frame, const ros::Time stamp)
{
  bool gotTransform = false;
  ros::Time before = ros::Time::now();
  if (!listener_->waitForTransform(parent_frame, child_frame, stamp, ros::Duration(0.5)))
  {
    ROS_ERROR("DIDNOT GET TRANSFORM %s %s at %f", parent_frame.c_str(), child_frame.c_str(), stamp.toSec());
    return false;
  }
  /* 得到parent_frame到 child_frame的坐标变换，存入trans中 */
  try
  {
    gotTransform = true;
    listener_->lookupTransform(parent_frame, child_frame, stamp, trans);
  }
  catch (tf::TransformException &ex)
  {
    gotTransform = false;
    ROS_ERROR("DIDNOT GET TRANSFORM %s %s,reason:%s", parent_frame.c_str(), child_frame.c_str(), ex.what()); // exception inherite from runtime_error,const char *
  }
  return gotTransform;
}



/**
* 将参考激光雷达信息转换为点云
**/
void ScanMapIcp::ReflaserScanSubCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  // ROS_INFO("I'm in the reflasercallback.");
  projector_->projectLaser(*msg, ref_cloud_in_scan_);
  // 0810注释　scan_after_transform_pub_.publish(ref_cloud_in_scan_);
  // ref_cloud_in_scan_输出没问题！！！
  // ROS_INFO("We get cloud_in_scan_! which means the projectLaser function  runs!");
  has_scan_ = false;
  got_transform_ = false;
  ROS_DEBUG("judge whether map to cloud transform found!");
  if (!listener_->waitForTransform("/map", ref_cloud_in_scan_.header.frame_id,\
                                   ref_cloud_in_scan_.header.stamp, ros::Duration(0.1)))
  // 等待laser_scan坐标系转换到map坐标系的坐标变换
  {
    ROS_WARN("no map to refcloud transform found!");
    scan_callback_mutex_.unlock();
    return;
  }
  ROS_DEBUG("judge whether map to base transform found!");
  if (!listener_->waitForTransform("/map", "/base_link", ref_cloud_in_scan_.header.stamp, ros::Duration(0.05)))
  // 等待base_link坐标系转换到map坐标系的坐标变换 
  {
    ROS_WARN("no map to base transform found!");
    scan_callback_mutex_.unlock();
    return;
  }
  // ROS_INFO("ready to transformScanToPointcloud");
  // 将激光的数据laserscan转换为pointcloud
  while (!got_transform_ && (ros::ok()))
  {
    try
    {
      got_transform_ = true;
      /**
      * 将位于激光坐标系下的激光点云转换到地图坐标系下
      * target frame = "map"
      * cloud in = cloud_in_scan
      * cloud_out = cloud_in_map
      * cloud_in_scan的类型为 sensor_msgs::PointCloud, frame为laser_scan frame(hokuyo link)
      * cloud_in_map的类型为 sensor_msgs::PointCloud，在激光坐标系中的数据数据在地图坐标系上的表示
      **/
      listener_->transformPointCloud("/map", ref_cloud_in_scan_, ref_cloud_in_map_);  
      // ref_cloud_in_scan_输出没问题！！！
      // ROS_INFO("transform cloud to map!");
    }
    catch(...)
    {
      got_transform_ = false;
      ROS_WARN("DIDNT GET TRANSFORM");
    }
  }
  for(size_t i = 0; i < ref_cloud_in_map_.points.size(); ++i)
  {
    ref_cloud_in_map_.points[i].z = 0;
  }
  got_transform_ = false;
 // tf::StampedTransform oldPose;
 // get base and map transform
  while (!got_transform_ && ros::ok())
  {
    try
    {
      got_transform_ = true;
      /**
      * 实现base_link到map的转换，变换矩阵存储在old_pose_中
      * target frame -----> map; source frame -----> base_link
      **/
      listener_->lookupTransform("/map", "/base_link", ref_cloud_in_scan_.header.stamp, old_pose_); 
      ROS_DEBUG("map->base_link's transform!");
    }
    catch(tf::TransformException &ex)
    {
      ROS_WARN("DIDNT GET TRANSFORM,%s", ex.what());
      got_transform_ = false;
    }
  }
}




/**
 * @brief ScanMapIcp::laserScanSubCallback, laser callback function
 * @param msg
 */
void ScanMapIcp::laserScanSubCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  ROS_INFO("HEAR START THE LASERSCANSUBCALLBACK FUNCTION: ");
  ROS_INFO("get laser!");
  if (!has_map_)
  {
    ROS_WARN("waiting for map to be published......");
    return;
  }
  ROS_INFO("has map!");
  scan_in_time_ = msg->header.stamp;  /* 激光数据进入的时间 */
  ros::Time time_received = ros::Time::now(); /* 程序已经运行的时间*/
  if (scan_in_time_ - last_proccessed_scan_ < ros::Duration(1 / scan_rate_))
  /* 时间太短，没有获得一帧完整的激光扫描 */
  {
    ROS_WARN("rejected scan,last %f,this %f", last_proccessed_scan_.toSec(), scan_in_time_.toSec());
    return;
  }
  // ROS_INFO("time long enough");
  if (!scan_callback_mutex_.try_lock())
    return;
  scan_age_ = ros::Time::now() - scan_in_time_;
  ROS_DEBUG("judge whether scan to old!");
  if (!use_sim_time_)
  {
    if (scan_age_.toSec() > age_threshold_)
    {
      ROS_WARN("SCAN SEEMS TO OLD (%f seconds %f threshold) scan time:%f now:%f", scan_age_.toSec(), age_threshold_,scan_in_time_.toSec(), ros::Time::now().toSec());
      scan_callback_mutex_.unlock();
      return;
    }
  }
  // ROS_INFO("use sim_time");
  ROS_DEBUG("judge whether get base pose at laser scan time!");
  //tf::StampedTransform base_at_laser;
  if (!getTransform(base_at_laser_, odom_frame_, "base_link", scan_in_time_))
  {
    ROS_WARN("Did not get base pose at laser scan time");
    scan_callback_mutex_.unlock();
    return;
  }
  // ROS_INFO("get transform");
  // if it satisfies conditions above, start to convert scan data to pointcloud
  // ROS_INFO("project laser!");

  //sensor_msgs::PointCloud cloud;
  //sensor_msgs::PointCloud cloudInMap;
  
  /**
  * Project a sensor_msgs::LaserScan into a sensor_msgs::PointCloud
  * msg的类型为 sensor_msgs::LaserScan
  * cloud_in_scan_的类型为 sensor_msgs::PointCloud
  * cloud_in_scan_的frame仍然是LaserScan的frame
  **/
  projector_->projectLaser(*msg, cloud_in_scan_);
  // ROS_INFO("We get cloud_in_scan_! which means the projectLaser function  runs!");
  has_scan_ = false;
  got_transform_ = false;
  ROS_DEBUG("judge whether map to cloud transform found!");
  if (!listener_->waitForTransform("/map", cloud_in_scan_.header.frame_id,\
                                   cloud_in_scan_.header.stamp, ros::Duration(0.05)))
  // 等待laser_scan坐标系转换到map坐标系的坐标变换
  {
    ROS_WARN("no map to lasercloud transform found!");
    scan_callback_mutex_.unlock();
    return;
  }
  ROS_DEBUG("judge whether map to base transform found!");
  if (!listener_->waitForTransform("/map", "/base_link", cloud_in_scan_.header.stamp, ros::Duration(0.05)))
  // 等待base_link坐标系转换到map坐标系的坐标变换 
  {
    ROS_WARN("no map to base transform found!");
    scan_callback_mutex_.unlock();
    return;
  }
  // ROS_INFO("ready to transformScanToPointcloud");
  // 将激光的数据laserscan转换为pointcloud
  transformScanToPointcloud();
  //将地图和激光数据都转化为点云，获得base和map的坐标变换------------->开始ICP
  runIcp();
  scan_callback_mutex_.unlock();
}

/**
 * @brief ScanMapIcp::transformScanToPointcloud,把激光数据转换为点云
 * 这个函数是有错误的！！！
 */
void ScanMapIcp::transformScanToPointcloud()
{
  while (!got_transform_ && (ros::ok()))
  {
    try
    {
      got_transform_ = true;
      /**
      * 将位于激光坐标系下的激光点云转换到地图坐标系下
      * target frame = "map"
      * cloud in = cloud_in_scan
      * cloud_out = cloud_in_map
      * cloud_in_scan的类型为 sensor_msgs::PointCloud, frame为laser_scan frame(hokuyo link)
      * cloud_in_map的类型为 sensor_msgs::PointCloud，在激光坐标系中的数据数据在地图坐标系上的表示
      **/
      listener_->transformPointCloud("/map", cloud_in_scan_, cloud_in_map_);  
      // ROS_INFO("transform cloud to map!");
    }
    catch(...)
    {
      got_transform_ = false;
      ROS_WARN("DIDNT GET TRANSFORM");
    }
  }

  for(size_t i = 0; i < cloud_in_map_.points.size(); ++i)
  {
    cloud_in_map_.points[i].z = 0;
  }
  got_transform_ = false;
 // tf::StampedTransform oldPose;
 // get base and map transform
  while (!got_transform_ && ros::ok())
  {
    try
    {
      got_transform_ = true;
      /**
      * 实现base_link到map的转换，变换矩阵存储在old_pose_中
      * target frame -----> map; source frame -----> base_link
      **/
      listener_->lookupTransform("/map", "/base_link", cloud_in_scan_.header.stamp, old_pose_); 
      ROS_DEBUG("map->base_link's transform!");
    }
    catch(tf::TransformException &ex)
    {
      ROS_WARN("DIDNT GET TRANSFORM,%s", ex.what());
      got_transform_ = false;
    }
  }
}
/**
 * @brief ScanMapIcp::runIcp，运行pcl中的icp
 */
void ScanMapIcp::runIcp()
{
  if ( got_transform_)
  {
    /**
    * 将sensor_msgs::PointCloud类型转换为sensor_msgs::PointCloud2类型
    * cloud_in_map是雷达扫描到的激光数据映射到map上，类型是sensor_msgs::PointCloud
    * cloud2 类型是sensor_msgs::PointCloud2
    **/
    sensor_msgs::convertPointCloudToPointCloud2(cloud_in_map_, cloud2_);
    sensor_msgs::convertPointCloudToPointCloud2(ref_cloud_in_map_, ref_cloud2_);
    //ref_cloud2_输出没问题
    has_scan_ = true;
    act_scan_ ++;
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> reg;
    /*Set the transformation epsilon (maximum allowable difference between two consecutive
     * transformations) in order for an optimization to be considered as having converged to the final
     * solution.
     */
    reg.setTransformationEpsilon(1e-6);
    /*Set the maximum distance threshold between two correspondent points in source <-> target. If the
     *distance is larger than this threshold, the points will be ignored in the alignment process.
     */
    reg.setMaxCorrespondenceDistance(0.5);
    //Set the maximum number of iterations the internal optimization should run for
    reg.setMaximumIterations(icp_num_iter_);

    PointCloudT::Ptr myMapCloud(new PointCloudT());
    PointCloudT::Ptr myRefCloud(new PointCloudT());
    PointCloudT::Ptr myScanCloud(new PointCloudT());
    //Convert a PCLPointCloud2 binary data blob into a pcl::PointCloud<T> object
    /**
    * map_cloud_ 为<sensor_msgs::PointCloud2>类型
    * cloud2_ 为<sensor_msgs::PointCloud2>类型
    **/
    source_pub.publish(cloud2_);
    target_pub.publish(ref_cloud2_);
    //pcl::fromROSMsg(*map_cloud_, *myMapCloud);
    
    pcl::fromROSMsg(ref_cloud2_, *myRefCloud);
    pcl::fromROSMsg(cloud2_, *myScanCloud);
    // 参考点云为myMapCloud， 输入点云为myScanCloud
    reg.setInputCloud(myScanCloud);
    reg.setInputTarget(myRefCloud);

    PointCloudT unused;
    int i = 0;
    //Call the registration algorithm which estimates the transformation and returns the transformed source
    reg.align(unused);

    // 获得ICP计算出的其次变换矩阵，存在transf中
    const Eigen::Matrix4f &transf = reg.getFinalTransformation();
    std::cout << transf<< std::endl;
    //轮椅仿真数据
    double ref_X = 6.3011;   //table的位置
    double ref_Y = -0.0943;  //table的位置
    // double ref_X = 6.498;   //table的位置
    // double ref_Y = -0.157;  //table的位置

    // double ref_X = -3.794;   //door的位置
    // double ref_Y = -0.284;  //door的位置

    //8.15新数据
    /**
    double ref_X = 6.1353;
    double ref_Y = -2.9885;
    **/
    /** 很久之前的仿真数据
    double ref_X = 6.2453;
    double ref_Y = -2.8831;**/
    
    /** 真机实验 
    double ref_X = -41.7171;
    double ref_Y = -6.0879;
    **/
    double now_X=0, now_Y=0;
    now_X = transf(0,0)*ref_X + transf(0,1)*ref_Y + transf(0,3);
    now_Y = transf(1,0)*ref_X + transf(1,1)*ref_Y + transf(1,3);
    now_X = ceil(now_X * 1000) * 0.001;
    now_Y = ceil(now_Y * 1000) * 0.001;
    std::cout << "now_X = " << now_X << ", " << "now_Y = " << now_Y << std::endl; 
    
    // tf::Transform t;
    matrixAsTransform(transf, t_);
    has_scan_transform_ = false;
    PointCloudT transformedCloud;
    /***
    * 输入点云为myScanCloud，输出点云为transformedCloud，依赖的坐标变换矩阵
    * 将输入点云通过得到的坐标变换矩阵，变换到参考坐标系下
    ***/ 
    pcl::transformPointCloud(*myScanCloud, transformedCloud, reg.getFinalTransformation()); 
    
    //compute inlier_percent of icp
    computeInlierPecent(transformedCloud);

    last_proccessed_scan_ = scan_in_time_ ;
    /**
    * transformedCloud 为pcl::PointCloud<pcl::PointXYZ>类型
    * cloud2transformed_ 为sensor_msgs::PointCloud2类型
    **/
    pcl::toROSMsg(transformedCloud, cloud2transformed_);
    //transformed_pointcloud.publish(cloud2transformed_); //发布变换后的点云
    has_scan_transform_ = true;    
    // compute real transform info, actually R and t in icp math model
    icp_result_.dist = sqrt((t_.getOrigin().x() * t_.getOrigin().x()) + (t_.getOrigin().y() * t_.getOrigin().y()));
    if (transf(0,1) < 0)
      icp_result_.angle_dist = t_.getRotation().getAngle();
    else
      icp_result_.angle_dist = -t_.getRotation().getAngle();
    /***
    * 对小车发送控制命令
    ***/
    // 计算小车当前相对于map坐标系的角度，然后对小车的角度进行修正
    listener_->lookupTransform("/map", "/base_link", cloud_in_scan_.header.stamp, est_pose_);
    double roll, pitch, yaw;
    tf::Matrix3x3(est_pose_.getRotation()).getEulerYPR(yaw, pitch, roll);
    std::cout << "yaw = " << yaw << std::endl;

    // std::cout << "Quaternion: " << tf::createQuaternionMsgFromYaw(yaw) << std::endl;
    //

    //　8.16　新增代码
    /**
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.stamp = ref_cloud2_.header.stamp;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = now_X;
    pose.pose.pose.position.y = now_Y;
    //t_ = t_* old_pose_;
    tf::Quaternion quat = est_pose_.getRotation();
    tf::quaternionTFToMsg(quat,pose.pose.pose.orientation); //将沿某个轴的旋转转化为四元数
    float poseFactor = 0.03;
    float rotFactor = 0.1;
    double cov = 1.5;
    pose.pose.covariance[6*0 + 0] = (cov * cov) * poseFactor;
    pose.pose.covariance[6*1 + 1] = (cov * cov) * poseFactor;
    pose.pose.covariance[6*5 + 5] = (M_PI /12.0 * M_PI /12.0) * rotFactor;
    dlf_icp_result.publish(pose);
    ros::shutdown();
    **/

    // 8.15 新增代码
    /**　失败的一次尝试,但是能够发送odom->map的tf变换了。8.16准备将检测出来的位置发送给amcl的initialpose
    nav_msgs::Odometry pose;
    pose.header.seq = count++;
    pose.header.stamp = ref_cloud2_.header.stamp;
    pose.header.frame_id = "map";
    pose.child_frame_id = "odom";
    pose.pose.pose.position.x = now_X;
    pose.pose.pose.position.y = now_Y;
    t_ = t_* old_pose_;
    tf::Quaternion quat = t_.getRotation();
    tf::quaternionTFToMsg(quat,pose.pose.pose.orientation); //将沿某个轴的旋转转化为四元数
    float poseFactor = 0.03;
    float rotFactor = 0.1;
    double cov = 1.5;
    pose.pose.covariance[6*0 + 0] = (cov * cov) * poseFactor;
    pose.pose.covariance[6*1 + 1] = (cov * cov) * poseFactor;
    pose.pose.covariance[6*5 + 5] = (M_PI /12.0 * M_PI /12.0) * rotFactor;
    dlf_icp_result.publish(pose);
    **/

    /** 自己控制时，将t_d_m.msg中的注释去掉，使用下述代码**/
    scan_map_icp::t_d_m error;
    error.header.stamp = ros::Time::now();
    error.header.frame_id = "hokuyo_link";
    error.linear_dist_x = ref_X - now_X ;
    error.linear_dist_y = ref_Y - now_Y;
    // std::cout << "linear_dist_y=" << error.linear_dist_y << std::endl;
    error.angle_dist_1 = yaw; //angle_dist_1表示先将机器人旋转到与世界坐标系对齐
    error.angle_dist_2 = icp_result_.angle_dist; //angle_dist_2表示机器人当前姿态与参考姿态的误差
    std::cout << "angle_error = " << icp_result_.angle_dist << std::endl;
    control_cmd.publish(error);
    

    rot_axis_ = t_.getRotation().getAxis();
    /**
    * old_pose_ 是base_link坐标系到map坐标系的齐次矩阵
    * t_ 是输入点云到目标点云的表换
    **/
    t_ = t_* msg_refpose;  //t_ = t_ * old_pose_;
    tf::StampedTransform base_after_icp;
    /**
    * parent_frame=odom_frame_, child_frame=base_link, transfrom存入base_after_icp 
    **/
    
    if(!getTransform(base_after_icp, odom_frame_, "base_link", ros::Time(0)))
    {
      ROS_WARN("DIDNT get base pose at now");
      scan_callback_mutex_.unlock();
      return;
    }
    else
    {
      // final compute's result
      // (base_at_laser_)的逆 * base_after_icp
      tf::Transform rel = base_at_laser_.inverseTimes(base_after_icp); // transform's times
      ROS_DEBUG("relative motion of robot while doing icp:%fcm %fdeg", rel.getOrigin().length(), rel.getRotation().getAngle() * 180 / M_PI);
      t_ = t_ * rel;
    }
    
    fitness_score_ = reg.getFitnessScore();
    icp_converge_ = reg.hasConverged();
    // publish icp result info
    publishIcpInfo();
 }
}

void ScanMapIcp::computeInlierPecent(PointCloudT &pc)
{
  icp_result_.inlier_percent = 0.0;
  std::vector<int> nn_indices(1); // save search results' index
  std::vector<float> nn_sqr_dists(1); // save search results' distance
  size_t inlier_num = 0;
  for (size_t k = 0; k < pc.points.size(); ++k)
  {
    //Search for all the nearest neighbors of the query point in a given radius
    if (map_tree_->radiusSearch(pc.points[k], icp_inlier_dist_, nn_indices, nn_sqr_dists, 1) != 0)
    {
      inlier_num += 1;
    }
  }
  if (pc.points.size() > 0)
  {
    icp_result_.inlier_percent = (double) inlier_num / (double) pc.points.size();
  }
}

void ScanMapIcp::publishIcpInfo()
{
  char msg_c_str[2048];
  /**
  *sprintf():将后面的字符串写入msg_c_str字符串数组中
  **/
  sprintf(msg_c_str, "inliers:%f(%f) scan_age %f (%f age_threshold) dist %f ang_dist %f axis(%f %f %f) \
         fitting %f(icp_fitness_threshold %f)", icp_result_.inlier_percent, icp_inlier_threshold_,\
          scan_age_.toSec(), age_threshold_, icp_result_.dist, icp_result_.angle_dist, rot_axis_.x(),\
          rot_axis_.y(), rot_axis_.z(), fitness_score_, icp_fitness_threshold_);
  ROS_INFO( "inliers:%f(%f) scan_age %f (%f age_threshold) dist %f ang_dist %f axis(%f %f %f) \
         fitting %f(icp_fitness_threshold %f)", icp_result_.inlier_percent, icp_inlier_threshold_,\
          scan_age_.toSec(), age_threshold_, icp_result_.dist, icp_result_.angle_dist, rot_axis_.x(),\
          rot_axis_.y(), rot_axis_.z(), fitness_score_, icp_fitness_threshold_);

  std_msgs::String str;
  str.data = msg_c_str;
  double cov = pose_covariance_trans_;
  //judge whether satisfy relocation
  //if ((act_scan_ - last_time_sent_) > update_age_threshold_ && (icp_result_.dist > dist_threshold_ || icp_result_.angle_dist > angle_threshold_) \
     && (icp_result_.inlier_percent > icp_inlier_threshold_) && (icp_result_.angle_dist < angle_upper_threshold_))
  if ((icp_result_.dist > dist_threshold_ || icp_result_.angle_dist > angle_threshold_) \
     && (icp_result_.inlier_percent < icp_inlier_threshold_) && (icp_result_.angle_dist < angle_upper_threshold_))
  {
    last_time_sent_ = act_scan_;
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = t_.getOrigin().x();
    pose.pose.pose.position.y = t_.getOrigin().y();
    tf::Quaternion quat = t_.getRotation();
    tf::quaternionTFToMsg(quat,pose.pose.pose.orientation); //将沿某个轴的旋转转化为四元数
    float poseFactor = 0.03;
    float rotFactor = 0.1;
    pose.pose.covariance[6*0 + 0] = (cov * cov) * poseFactor;
    pose.pose.covariance[6*1 + 1] = (cov * cov) * poseFactor;
    pose.pose.covariance[6*5 + 5] = (M_PI /12.0 * M_PI /12.0) * rotFactor;
    // ROS_INFO("i %i converged %i score: %f", icp_converge_, fitness_score_);
    ROS_INFO("publish a new initial pose, dist %f angle_dist %f,setting pose:%.3f %.3f [frame=%s] ", \
             icp_result_.dist, icp_result_.angle_dist, pose.pose.pose.position.x, pose.pose.pose.position.y,\
             pose.header.frame_id.c_str());
    if (call_icp_msg_ == "start")
    {
      ROS_WARN("publish a new initial pose, dist %f angle_dist %f,setting pose:%.3f %.3f [frame=%s] ", \
               icp_result_.dist, icp_result_.angle_dist, pose.pose.pose.position.x, pose.pose.pose.position.y,\
               pose.header.frame_id.c_str());
      // initial_pose_pub_.publish(pose);
    }
    str.data += "<< sent";
  }
  if (call_icp_msg_ == "start")
  {
    icp_run_info_pub_.publish(str);
  }
}
void ScanMapIcp::updateParams()
{
  params_were_updated_ = ros::Time::now();
  private_nh_.param("use_sim_time", use_sim_time_, false);
  private_nh_.param("icp_fitness_threshold", icp_fitness_threshold_, 100.0);
  private_nh_.param("age_threshold", age_threshold_, 1.0);
  private_nh_.param("angle_upper_threshold", angle_upper_threshold_, 1.0);
  private_nh_.param("angle_threshold", angle_threshold_, 0.01);
  private_nh_.param("update_age_threshold", update_age_threshold_, 1.0);
  private_nh_.param("dist_threshold", dist_threshold_, 0.01);
  private_nh_.param("icp_inlier_threshold", icp_inlier_threshold_, 0.88);
  private_nh_.param("icp_inlier_dist", icp_inlier_dist_, 0.01);
  private_nh_.param("icp_num_iter", icp_num_iter_, 500);
  private_nh_.param("pose_covariance_trans", pose_covariance_trans_, 0.5);
  private_nh_.param("scan_rate", scan_rate_, 15);
  if(scan_rate_ < .001)
  {
    scan_rate_ = .001;
  }
}
/*
void ScanMapIcp::baseMotionTypeSubCallback(const std_msgs::String::ConstPtr &data)
{
  if (data->data == "straight")
  {
    ROS_INFO("base is now go straight");
  }
  else if (data->data == "turn")
  {
    ROS_INFO("base is now turn!");
  }
  else if(data->data == "arc")
  {
    ROS_INFO("base is now go arc!");
  }
  else if (data->data == "static")
  {
    ROS_INFO("base is now static!");
  }
  else
  {}
  base_motion_type_ = data->data;
}*/
void ScanMapIcp::callIcpSubCallback(const std_msgs::String::ConstPtr &data)
{
  if (data->data == "start")
  {
    ROS_INFO("NOW, START TO CALL ICP!");
    call_icp_msg_ = data->data;
  }
  else if (data->data == "finish")
  {
    ROS_INFO("Now, FINSH CALL ICP!");
    call_icp_msg_ = data->data;
  }
  else
  {
    ROS_ERROR("INVALID MSG!");
  }
}
}
