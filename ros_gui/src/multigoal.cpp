#include "../include/ros_gui/multigoal.h"
#include "../build/ui_multigoal.h"

Multigoal::Multigoal(int argc, char** argv,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Multigoal),
    multinode(argc, argv)
{
    ui->setupUi(this);
    multinode.init();

    connect(this, SIGNAL(multigoal_signal()), &multinode, SLOT(multigoal_slot()));

}


Multigoal::~Multigoal()
{
    delete ui;
}

void Multigoal::on_pushButton_clicked(){
    emit setgoal_signal();
}

void Multigoal::on_pushButton_2_clicked(){
    emit multigoal_signal();
    this->hide();
}




