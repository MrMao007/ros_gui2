#include "../include/ros_gui/demo.h"
#include "../build/ui_demo.h"

Demo::Demo(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Demo)
{
    ui->setupUi(this);

}


Demo::~Demo()
{
    delete ui;
}

void Demo::on_pushButton_clicked(){
    if(this->ui->lineEdit->text().isEmpty()){
        QMessageBox::information(NULL, "消息", "需要确定示教点名称!!!", QMessageBox::Yes, QMessageBox::Yes);
        return;
    }
    system("gnome-terminal -x bash -c 'bash ~/bash/demostration.sh'");
    emit demostration_signal(this->ui->lineEdit->text());
    this->hide();
}






