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

#ifndef MULTINODE_H_
#define MULTINODE_H_

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
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32.h>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Ui {

/*****************************************************************************
** Class
*****************************************************************************/

class Multinode : public QThread {
    Q_OBJECT
public:
    Multinode(int argc, char** argv );
    virtual ~Multinode();
	bool init();
	void run();
    void callback(const geometry_msgs::PoseStampedConstPtr &goal_msg);
    void send_multigoal();
    std::vector<geometry_msgs::PoseStamped> goal_vec;

public Q_SLOTS:
    void multigoal_slot();

private:
	int init_argc;
	char** init_argv;
    ros::Subscriber goal_sub;
};

}  // namespace ros_gui

#endif /* ros_gui_QNODE_HPP_ */
