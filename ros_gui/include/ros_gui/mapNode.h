/**
 * @file /include/ros_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef MAPNODE_H_
#define MAPNODE_H_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QFile>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <custom_msgs/Obstacles.h>
#include <custom_msgs/Form.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32.h>
#include <QtConcurrent/QtConcurrent>
#include <opencv2/opencv.hpp>
//#include <qfiledialog.h>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Ui{

/*****************************************************************************
** Class
*****************************************************************************/

class MapNode : public QThread {
    Q_OBJECT
public:
    MapNode(int argc, char** argv );
    virtual ~MapNode();
    bool init();
    void run();
    void startpCallback(const geometry_msgs::PointStampedConstPtr &sp);
    void endpCallback(const geometry_msgs::PointStampedConstPtr &ep);
    void line_startpCallback(const geometry_msgs::PointStampedConstPtr &sp);
    void line_endpCallback(const geometry_msgs::PointStampedConstPtr &ep);
    void pointpCallback(const geometry_msgs::PointStampedConstPtr &sp);
    void semanticpCallback(const geometry_msgs::PoseStampedConstPtr &sp);
    void goalCallback(const geometry_msgs::PoseStampedConstPtr &goal);
    void coarseCallback(const geometry_msgs::PoseStampedConstPtr &goal);
    void pathCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &traj);
    void routeCallback(const nav_msgs::PathConstPtr &route);
    void refscanCallback(const sensor_msgs::LaserScanConstPtr &scan);
    void dock_stateCallback(const std_msgs::Int8ConstPtr &state);
    void send_multigoal();
    void send_door_front();
    void send_door_in();
    void send_door_out();
    void send_dock();
    void readFile(std::string);
    geometry_msgs::PoseStamped coarsetofine(const geometry_msgs::PoseStampedConstPtr &goal);
    void WriteLaser(const sensor_msgs::LaserScan scan);


    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::MarkerArray semantic_markerarray;
    visualization_msgs::MarkerArray goal_markerarray;
    visualization_msgs::MarkerArray goalid_markerarray;
    visualization_msgs::MarkerArray idarray;
    visualization_msgs::MarkerArray path_markerarray;
    custom_msgs::Obstacles obstacles;


    double startp[2];
    double endp[2];
    double line_startp[2];
    double line_endp[2];
    double pointp[2];
    double semanticp[2];
    geometry_msgs::Pose last_pose;
    std::vector<geometry_msgs::Pose> path;
    std::vector<geometry_msgs::PoseStamped> semantic_vec;
    std::vector<geometry_msgs::PoseStamped> goal_vec;
    sensor_msgs::LaserScan got_laser;
    sensor_msgs::LaserScan ref_laser;
    geometry_msgs::PoseWithCovarianceStamped ref_goal;
    int id = 0;
    int semantic_id = 0;
    int goal_id = 0;
    int path_id = 0;
    int empty_flag = 1;
    int record_flag = 0;
    int track_flag = 0;
    int demo_flag = 0;
    int dock_ready_flag = 0;
    int dock_state = 0;
    QString demo_name;
    QString dock_name;

    void add_path_makerarray(geometry_msgs::Pose last);
    void clear_path_markerarray();

signals:
    void startpUpdated(double x, double y);
    void endpUpdated(double x, double y);
    void line_startpUpdated(double x, double y);
    void line_endpUpdated(double x, double y);
    void pointpUpdated(double x, double y);
    void semanticpUpdated(double x,double y);
    void door_front_ready_signal();
    void door_in_ready_signal();
    void door_out_ready_signal();
    void dock_ready_signal();
    void demostration_ready_signal(QString demo_n);
    void ros_shutdown();

public Q_SLOTS:

    void marker_slot();
    void line_marker_slot();
    void point_marker_slot(double radius);
    void delete_slot(int id);
    void semantic_slot(std::string semantic);
    void multigoal_slot();
    void record_path_slot();
    void save_path_slot();
    void track_slot();
    void track_shut_slot();
    void door_front_slot();
    void door_in_slot();
    void door_out_slot();
    void vicon_slot();
    void tea_slot();
    void dock_slot(QString dock_n);
    void demostration_slot(QString demo_n);
    void track_2d_slot();

private:
    int init_argc;
    char** init_argv;
    //sensor_msgs::LaserScan scan_;
    geometry_msgs::PoseWithCovarianceStamped pose_;
    //ros::Publisher text_pub;
    //ros::Subscriber power_sub;
    ros::Subscriber startp_sub;
    ros::Subscriber endp_sub;
    ros::Subscriber line_startp_sub;
    ros::Subscriber line_endp_sub;
    ros::Subscriber pointp_sub;
    ros::Subscriber semanticp_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber coarse_sub;
    ros::Subscriber path_sub;
    ros::Subscriber route_sub;
    ros::Subscriber refscan_sub;
    ros::Subscriber dock_state_sub;

    ros::Publisher markerarray_pub;
    ros::Publisher semantic_markerarray_pub;
    ros::Publisher goalmarker_pub;
    ros::Publisher goalid_pub;
    ros::Publisher idarray_pub;
    ros::Publisher obstacle_pub;
    ros::Publisher fine_pub;
    ros::Publisher pathmarker_pub;
    ros::Publisher routemarker_pub;
    ros::Publisher refscan_pub;
    ros::Publisher goal_pub;


};

}  // namespace ros_gui

#endif /* ros_gui_QNODE_HPP_ */
