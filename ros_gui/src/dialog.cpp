#include "../include/ros_gui/dialog.h"
#include "../build/ui_dialog.h"

Dialog::Dialog(int argc, char** argv, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Dialog),
    mapnode(argc, argv)//,
    //delete_ui(new Ui::Delete)
{
    ui->setupUi(this);

    connect(&mapnode, SIGNAL(startpUpdated(double,double)), this, SLOT(startpUpdated_slot(double,double)));
    connect(&mapnode, SIGNAL(endpUpdated(double,double)), this, SLOT(endpUpdated_slot(double,double)));
    connect(&mapnode, SIGNAL(line_startpUpdated(double,double)), this, SLOT(line_startpUpdated_slot(double,double)));
    connect(&mapnode, SIGNAL(line_endpUpdated(double,double)), this, SLOT(line_endpUpdated_slot(double,double)));
    connect(&mapnode, SIGNAL(pointpUpdated(double,double)), this, SLOT(pointpUpdated_slot(double,double)));
    connect(this, SIGNAL(markersignal()), &mapnode, SLOT(marker_slot()));
    connect(this, SIGNAL(line_markersignal()), &mapnode, SLOT(line_marker_slot()));
    connect(this, SIGNAL(point_markersignal(double)), &mapnode, SLOT(point_marker_slot(double)));


    mapnode.init();
}


Dialog::~Dialog()
{
    delete ui;
}

void Dialog::on_pushButton_clicked(){
    emit startp();
}

void Dialog::on_pushButton_2_clicked(){
    emit endp();
}

void Dialog::on_pushButton_3_clicked(){
    emit markersignal();
}

void Dialog::on_pushButton_4_clicked(){
    this->hide();
}

void Dialog::on_pushButton_13_clicked(){
    emit line_startp();
}

void Dialog::on_pushButton_14_clicked(){
    emit line_endp();
}

void Dialog::on_pushButton_15_clicked(){
    emit line_markersignal();
}

void Dialog::on_pushButton_16_clicked(){
    this->hide();
}


void Dialog::on_pushButton_20_clicked(){
    emit pointp();
}

void Dialog::on_pushButton_17_clicked(){
    double radius = ui->lineEdit_5->text().toDouble();
    emit point_markersignal(radius);
}

void Dialog::on_pushButton_18_clicked(){
    this->hide();
}

void Dialog::startpUpdated_slot(double x, double y){
    ui->lineEdit->setText(QString::number(x,10,2));
    ui->lineEdit_2->setText(QString::number(y,10,2));
}

void Dialog::endpUpdated_slot(double x, double y){
    ui->lineEdit_3->setText(QString::number(x,10,2));
    ui->lineEdit_4->setText(QString::number(y,10,2));
}

void Dialog::line_startpUpdated_slot(double x, double y){
    ui->lineEdit_13->setText(QString::number(x,10,2));
    ui->lineEdit_14->setText(QString::number(y,10,2));
}

void Dialog::line_endpUpdated_slot(double x, double y){
    ui->lineEdit_15->setText(QString::number(x,10,2));
    ui->lineEdit_16->setText(QString::number(y,10,2));
}

void Dialog::pointpUpdated_slot(double x, double y){
    ui->lineEdit_17->setText(QString::number(x,10,2));
    ui->lineEdit_18->setText(QString::number(y,10,2));
}

