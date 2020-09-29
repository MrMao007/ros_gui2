#include "../include/ros_gui/mainwindow.h"
#include "../include/ros_gui/arm.h"
#include <QApplication>
#include <QString>


int main(int argc, char *argv[])
{
    //ros::init(argc,argv,"ros_gui");
    //ros::start(); // explicitly needed since our nodehandle is going out of scope.
    // Add your ros communications here.
    //ros::init(argc, argv, "main");
    QApplication a(argc, argv);
    MainWindow *w = new MainWindow(argc, argv);
    a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
    //RoMap r;
    //Dialog d(argc,argv);
    //ros::NodeHandle n;
    //ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/front_scan", 1000, &Dialog::callback, &d);
    w->setWindowTitle(QString::fromLocal8Bit("机器人控制中心"));
    //ros::Rate loop_rate(1000);

    //w.move((a.desktop()->width() - w.width()) / 2, (a.desktop()->height() - w.height()) / 2);
    w->show();
    //r.show();
    //d.show();

    QFile styleFile(":/style/style.qss");
    if(styleFile.open(QIODevice::ReadOnly))
    {
        qDebug("open success");
        QString setStyleSheet(styleFile.readAll());
        a.setStyleSheet(setStyleSheet);
        styleFile.close();
    }
    else
    {
        qDebug("Open failed");
    }
    int r = a.exec();
    //ros::shutdown();
    return r;
}

