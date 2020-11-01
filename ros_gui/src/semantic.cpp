#include "../include/ros_gui/semantic.h"
#include "../build/ui_semantic.h"

Semantic::Semantic(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Semantic)
{
    ui->setupUi(this);

}


Semantic::~Semantic()
{
    delete ui;
}

void Semantic::on_pushButton_clicked(){
    QString semantic_text = ui->lineEdit->text();
    std::string temp = semantic_text.toStdString();
    this->hide();
    emit semantic_signal(temp);
}

void Semantic::on_pushButton_2_clicked(){
    emit semanticp();
}





