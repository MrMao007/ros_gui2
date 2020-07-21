#include "../include/ros_gui/delete.h"
#include "../build/ui_delete.h"

Delete::Delete(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Delete)
{
    ui->setupUi(this);

}


Delete::~Delete()
{
    delete ui;
}

void Delete::on_pushButton_clicked(){
    QString id_str = ui->lineEdit->text();
    int id = id_str.toInt();
    emit delete_signal(id);
}





