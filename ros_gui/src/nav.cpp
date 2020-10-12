#include "../include/ros_gui/nav.h"
#include "../build/ui_nav.h"

Nav::Nav(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Nav)
{
    ui->setupUi(this);

}


Nav::~Nav()
{
    delete ui;
}

void Nav::on_pushButton_clicked(){
    ui->pushButton->setEnabled(false);
    switch (door_state) {
    case 1:
        emit door_front_signal();
        break;
    case 2:
        emit door_in_signal();
        break;
    case 3:
        emit door_out_signal();
        break;
    default:
        break;
    }
}

void Nav::on_pushButton_2_clicked(){
;
}

void Nav::on_pushButton_3_clicked(){
    ;
}

void Nav::door_front_ready_slot()
{
    ui->pushButton->setEnabled(true);
    ui->pushButton->setText("进门");
    door_state = 2;
}

void Nav::door_in_ready_slot()
{
    ui->pushButton->setEnabled(true);
    ui->pushButton->setText("出门");
    door_state = 3;
}

void Nav::door_out_ready_slot()
{
    ui->pushButton->setEnabled(true);
    ui->pushButton->setText("电梯门前");
    door_state = 1;
}




