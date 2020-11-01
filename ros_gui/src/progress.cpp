#include "../include/ros_gui/progress.h"
#include "../build/ui_progress.h"

Progress::Progress(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Progress)
{
    ui->setupUi(this);
    ui->progressBar->setMinimum(0);
    ui->progressBar->setMaximum(100);

}


Progress::~Progress()
{
    delete ui;
}



void Progress::setProgress(float per, QString format){
    ui->progressBar->setValue(per);
    ui->progressBar->setFormat(format);
}




