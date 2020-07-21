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
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <custom_msgs/Obstacles.h>
#include <custom_msgs/Form.h>
#include <move_base_msgs/MoveBaseAction.h>



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
    void semanticpCallback(const geometry_msgs::PointStampedConstPtr &sp);
    void goalCallback(const geometry_msgs::PoseStampedConstPtr &goal);



    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::MarkerArray semantic_markerarray;
    visualization_msgs::MarkerArray goal_markerarray;
    visualization_msgs::MarkerArray goalid_markerarray;
    visualization_msgs::MarkerArray idarray;
    custom_msgs::Obstacles obstacles;


    double startp[2];
    double endp[2];
    double line_startp[2];
    double line_endp[2];
    double pointp[2];
    double semanticp[2];
    int id = 0;
    int semantic_id = 0;
    int goal_id = 0;
signals:
    void startpUpdated(double x, double y);
    void endpUpdated(double x, double y);
    void line_startpUpdated(double x, double y);
    void line_endpUpdated(double x, double y);
    void pointpUpdated(double x, double y);
    void semanticpUpdated(double x,double y);

public Q_SLOTS:

    void marker_slot();
    void line_marker_slot();
    void point_marker_slot(double radius);
    void delete_slot(int id);
    void semantic_slot(std::string semantic);

private:
    int init_argc;
    char** init_argv;

    //ros::Publisher text_pub;
    //ros::Subscriber power_sub;
    ros::Subscriber startp_sub;
    ros::Subscriber endp_sub;
    ros::Subscriber line_startp_sub;
    ros::Subscriber line_endp_sub;
    ros::Subscriber pointp_sub;
    ros::Subscriber semanticp_sub;
    ros::Subscriber goal_sub;
    ros::Publisher markerarray_pub;
    ros::Publisher semantic_markerarray_pub;
    ros::Publisher goalmarker_pub;
    ros::Publisher goalid_pub;
    ros::Publisher idarray_pub;
    ros::Publisher obstacle_pub;


};

}  // namespace ros_gui

#endif /* ros_gui_QNODE_HPP_ */
