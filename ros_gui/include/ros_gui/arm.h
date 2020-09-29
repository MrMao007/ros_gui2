#ifndef ARM_H
#define ARM_H

#include <QWidget>
#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include<rviz/tool_manager.h>
#include <rviz/view_manager.h>
#include <QMutex>
#include "qnode.h"

namespace Ui {
class arm;
}

class arm : public QWidget
{
    Q_OBJECT

public:
    arm(int argc, char** argv, QWidget *parent = 0);
    ~arm();

    void ros_init(int argc,char **argv);

     Ui::QNode qnode;

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();



public Q_SLOTS:
    void updateJoint();
/*
    void displayCamera(const QImage& image);

    void updateLogcamera();
*/
signals:
    void sendsignal();


private:
    Ui::arm *ui;
    rviz::VisualizationManager *manager_;
    rviz::RenderPanel *render_panel_;
    rviz::ViewManager* viewManager;
    QImage qimage_;
    mutable QMutex qimage_mutex_;
};

#endif // ARM_H
