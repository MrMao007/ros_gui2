#include "../include/ros_gui/arm.h"
#include "../build/ui_arm.h"
#define pi 3.1415926


arm::arm(int argc, char** argv, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::arm),
    qnode(argc, argv)
{
    ui->setupUi(this);
    ui->label_17->setText("Disconnected!");
    ui->label_17->setStyleSheet("color:red");

    ros_init(argc,argv);
    render_panel_=new rviz::RenderPanel;
    ui->verticalLayout_3->addWidget(render_panel_);
    manager_=new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(manager_->getSceneManager(),manager_);
    manager_->initialize();
    manager_->startUpdate();

    this->viewManager = manager_->getViewManager();
    this->viewManager->setRenderPanel(render_panel_);
    this->viewManager->getCurrent()->subProp("Distance")->setValue("3.0");
    this->viewManager->getCurrent()->subProp("Focal Point")->setValue("-0.29208;-0.30049;0.44673");
    manager_->removeAllDisplays();

    manager_->setFixedFrame("base_link");
    rviz::Display *robot=manager_->createDisplay("rviz/RobotModel","adjustable robot",true);
    ROS_ASSERT(robot!=NULL);
    robot->subProp("Robot Description")->setValue("robot_description");
    qnode.init();
    connect(&qnode, SIGNAL(jointUpdated()), this, SLOT(updateJoint()));
    //connect(&qnode, SIGNAL(loggingCamera()), this, SLOT(updateLogcamera()));


}

arm::~arm()
{
    delete ui;
}

void arm::ros_init(int argc,char **argv)
{
  ros::init(argc,argv,"arm",ros::init_options::AnonymousName);
}

void arm::on_pushButton_clicked(){
    emit sendsignal();
    this->hide();
}

void arm::on_pushButton_2_clicked(){
    system("gnome-terminal -x bash -c 'bash ~/bash/arm.sh'");
    ui->label_17->setText("Connected!");
    ui->label_17->setStyleSheet("color:green");
}

void arm::on_pushButton_3_clicked(){
    system("gnome-terminal -x bash -c 'bash ~/bash/arm_shut.sh'");
    ui->label_17->setText("Disconnected!");
    ui->label_17->setStyleSheet("color:red");
}

void arm::updateJoint(){
    ui->lineEdit->setText(QString::number(qnode.joint_states[0]*180/pi, 10, 2)+"°");
    ui->lineEdit_2->setText(QString::number(qnode.joint_states[1]*180/pi, 10, 2)+"°");
    ui->lineEdit_3->setText(QString::number(qnode.joint_states[2]*180/pi, 10, 2)+"°");
    ui->lineEdit_4->setText(QString::number(qnode.joint_states[3]*180/pi, 10, 2)+"°");
    ui->lineEdit_5->setText(QString::number(qnode.joint_states[4]*180/pi, 10, 2)+"°");
    ui->lineEdit_6->setText(QString::number(qnode.joint_states[5]*180/pi, 10, 2)+"°");
    ui->lineEdit_7->setText(QString::number(qnode.joint_states[6]*180/pi, 10, 2)+"°");
}
/*
void arm::displayCamera(const QImage &image)
{
    qimage_mutex_.lock();
    qimage_ = image.copy();
    ui->camera_label->setPixmap(QPixmap::fromImage(qimage_));
    ui->camera_label->resize(ui->camera_label->pixmap()->size());
    qimage_mutex_.unlock();
}

void arm::updateLogcamera()
{
  displayCamera(qnode.image);
}
*/
