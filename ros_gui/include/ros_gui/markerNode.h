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

#ifndef MARKERNODE_H_
#define MARKERNODE_H_

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
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Ui {

/*****************************************************************************
** Class
*****************************************************************************/

class MarkerNode : public QThread {
    Q_OBJECT
public:
    MarkerNode(int argc, char** argv );
    virtual ~MarkerNode();
    bool init();
    void run();
    void callback(const geometry_msgs::TwistConstPtr &cmd_vel);
    void powerCallback(const std_msgs::Float32ConstPtr &power);
    void flagCallback(const std_msgs::Float32ConstPtr &flag);
    void tempCallback(const std_msgs::Float32ConstPtr &temp);

    visualization_msgs::MarkerArray markerarray;
    visualization_msgs::Marker lmarker;
    visualization_msgs::Marker amarker;
    visualization_msgs::Marker tmarker;
/*signals:
    void jointUpdated();*/

signals:
    void power(float p);
    void power_flag(float f);
    void temp(float t);

private:
    int init_argc;
    char** init_argv;
    ros::Subscriber twist_sub;
    ros::Publisher lmarker_pub;
    ros::Publisher amarker_pub;
    ros::Publisher text_pub;
    ros::Subscriber power_sub;
    ros::Subscriber power_flag_sub;
    ros::Subscriber temp_sub;


};

}  // namespace ros_gui

#endif /* ros_gui_QNODE_HPP_ */
