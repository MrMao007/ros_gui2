#include "../include/ros_gui/dock.h"
#include "../build/ui_dock.h"

Dock::Dock(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dock)
{
    ui->setupUi(this);
    std::string pattern = "/home/mty/bash/*.dat";
    std::vector<cv::String> fn;
    cv::glob(pattern, fn);
    for(int i = 0; i < fn.size(); i++){
        std::string temp = fn[i];
        int pos = temp.find_last_of('/');
        std::string sfilename = temp.substr(pos+1);
        pos = sfilename.find_last_of('.');
        QString temp_dock = QString::fromStdString(sfilename.substr(0,pos));
        dock_names.push_back(temp_dock);
        this->ui->comboBox->addItem(temp_dock);
    }


}


Dock::~Dock()
{
    delete ui;
}

void Dock::on_pushButton_clicked(){
    QString dock_n = this->ui->comboBox->currentText();
    emit dock_signal(dock_n);
    this->hide();
}

void Dock::demostration_ready_slot(QString demo_n){
    dock_names.push_back(demo_n);
    this->ui->comboBox->addItem(demo_n);
}





