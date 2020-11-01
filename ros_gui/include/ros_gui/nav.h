#ifndef NAV_H
#define NAV_H

#include <QDialog>
#include <vector>
#include <QString>
#include <geometry_msgs/PoseStamped.h>

namespace Ui {
class Nav;
}

class Nav : public QDialog
{
    Q_OBJECT

public:
    explicit Nav(QWidget *parent = 0);
    ~Nav();
    int door_state = 1;

signals:
    void door_front_signal();

    void door_in_signal();

    void door_out_signal();
	
    void vicon_signal();
    
    void tea_signal();

public Q_SLOTS:
    void door_front_ready_slot();

    void door_in_ready_slot();

    void door_out_ready_slot();

private slots:

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

private:
    Ui::Nav *ui;
    //ros::Publisher goal_pub;


};

#endif // SEMANTIC_H
