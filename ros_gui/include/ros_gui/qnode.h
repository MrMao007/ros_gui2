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

#ifndef QNODE_H_
#define QNODE_H_

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
#include <QStringListModel>
#include <sensor_msgs/JointState.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace Ui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
    void callback(const sensor_msgs::JointStateConstPtr &joint_msg);
    void img_callback(const sensor_msgs::ImageConstPtr& img);
    double joint_states[7];
    QImage image;

signals:
    void jointUpdated();
    void loggingCamera();

private:
	int init_argc;
	char** init_argv;
    ros::Subscriber joint_sub;
    image_transport::Subscriber image_sub;
    cv::Mat img;
};

}  // namespace ros_gui

#endif /* ros_gui_QNODE_HPP_ */
