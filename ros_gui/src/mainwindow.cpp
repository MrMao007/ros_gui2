#include "../include/ros_gui/mainwindow.h"
#include "../build/ui_mainwindow.h"
#include <math.h>
#include <unistd.h>
#include <QCloseEvent>

#define pi 3.1415926


MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    markernode(argc, argv)
{

    ui->setupUi(this);


    this->statusLabel = new QLabel;

    this->arm_ui = new arm(argc, argv);
    this->dialog_ui = new Dialog(argc, argv);
    this->delete_ui = new Delete();
    this->semantic_ui = new Semantic();
    this->multigoal_ui = new Multigoal(argc, argv);
    this->demo_ui = new Demo();
    this->dock_ui = new Dock();
    this->nav_ui = new Nav();

    this->infoLabel = new QLabel;
    infoLabel->setMinimumSize(300,20); //设置标签最小尺寸
    infoLabel->setFrameShape(QFrame::WinPanel); //设置标签形状
    infoLabel->setFrameShadow(QFrame::Sunken); //设置标签阴影
    statusBar()->addWidget(infoLabel);
    infoLabel->setStyleSheet("color:green");
    infoLabel->setText("正常");

    QFont font("Microsoft YaHei", 12, 50);
    ui->label_5->setFont(font);
    ui->label_6->setFont(font);
    ui->label_7->setFont(font);
    ui->label_8->setFont(font);
    ui->label_9->setFont(font);
    ui->label_10->setFont(font);
    ui->label_11->setFont(font);
    ui->label_12->setFont(font);
    ui->label_13->setFont(font);
    ui->label_14->setFont(font);
    ui->label_15->setFont(font);
    ui->pushButton_23->setFont(font);
    markernode.init();

    ros::init(argc,argv,"ros_gui",ros::init_options::AnonymousName);
    render_panel_=new rviz::RenderPanel;
    ui->verticalLayout->addWidget(render_panel_);

    manager_=new rviz::VisualizationManager(render_panel_);
    tool_manager_=manager_->getToolManager();
    viewManager_ = manager_->getViewManager();
    //viewManager_->getCurrent()->subProp("Target Frame")->setValue("/map");

    render_panel_->initialize(manager_->getSceneManager(),manager_);
    manager_->initialize();
    tool_manager_->initialize();
    manager_->startUpdate();

    manager_->removeAllDisplays();

    //connect(ui->radioButton, SIGNAL(toggled(bool)), this, SLOT(on(bool)));
    connect(&markernode, SIGNAL(power(float)), this, SLOT(power_slot(float)));
    connect(&markernode, SIGNAL(power_flag(float)), this, SLOT(flag_slot(float)));
    connect(&markernode, SIGNAL(temp(float)), this, SLOT(temp_slot(float)));
    connect(this, SIGNAL(record_path_signal()), &(this->dialog_ui->mapnode), SLOT(record_path_slot()));
    connect(this, SIGNAL(save_path_signal()), &(this->dialog_ui->mapnode), SLOT(save_path_slot()));
    connect(this, SIGNAL(track_signal()), &(this->dialog_ui->mapnode), SLOT(track_slot()));
    connect(this, SIGNAL(track_shut_signal()), &(this->dialog_ui->mapnode), SLOT(track_shut_slot()));
    connect(this, SIGNAL(track_2d_signal()), &(this->dialog_ui->mapnode), SLOT(track_2d_slot()));
    connect(dialog_ui,SIGNAL(startp()),this,SLOT(startp_slot()));
    connect(dialog_ui,SIGNAL(endp()),this,SLOT(endp_slot()));
    connect(dialog_ui,SIGNAL(markersignal()),this,SLOT(markersignal_slot()));
    connect(dialog_ui,SIGNAL(line_startp()),this,SLOT(line_startp_slot()));
    connect(dialog_ui,SIGNAL(line_endp()),this,SLOT(line_endp_slot()));
    connect(dialog_ui,SIGNAL(line_markersignal()),this,SLOT(line_markersignal_slot()));
    connect(dialog_ui,SIGNAL(pointp()),this,SLOT(pointp_slot()));
    connect(dialog_ui,SIGNAL(point_markersignal(double)),this,SLOT(point_markersignal_slot()));
    connect(semantic_ui,SIGNAL(semanticp()),this,SLOT(semanticp_slot()));
    connect(semantic_ui,SIGNAL(semantic_signal(std::string)),&(this->dialog_ui->mapnode), SLOT(semantic_slot(std::string)));
    connect(delete_ui,SIGNAL(delete_signal(int)),&(this->dialog_ui->mapnode), SLOT(delete_slot(int)));
    connect(multigoal_ui, SIGNAL(setgoal_signal()), this, SLOT(setgoal_slot()));
    connect(this->multigoal_ui, SIGNAL(multigoal_signal()), &(this->dialog_ui->mapnode), SLOT(multigoal_slot()));
    connect(nav_ui, SIGNAL(door_front_signal()), &(this->dialog_ui->mapnode), SLOT(door_front_slot()));
    connect(nav_ui, SIGNAL(door_in_signal()), &(this->dialog_ui->mapnode), SLOT(door_in_slot()));
    connect(nav_ui, SIGNAL(door_out_signal()), &(this->dialog_ui->mapnode), SLOT(door_out_slot()));
    connect(&(this->dialog_ui->mapnode), SIGNAL(door_front_ready_signal()), nav_ui, SLOT(door_front_ready_slot()));
    connect(&(this->dialog_ui->mapnode), SIGNAL(door_in_ready_signal()), nav_ui, SLOT(door_in_ready_slot()));
    connect(&(this->dialog_ui->mapnode), SIGNAL(door_out_ready_signal()), nav_ui, SLOT(door_out_ready_slot()));
    connect(&(this->dialog_ui->mapnode), SIGNAL(dock_ready_signal()), this, SLOT(dock_ready_slot()));
    connect(&(this->dialog_ui->mapnode), SIGNAL(dock_ready_signal()), this, SLOT(dock_ready_slot2()));
    connect(demo_ui, SIGNAL(demostration_signal(QString)), &(this->dialog_ui->mapnode), SLOT(demostration_slot(QString)));
    connect(dock_ui, SIGNAL(dock_signal(QString)), &(this->dialog_ui->mapnode), SLOT(dock_slot(QString)));
    connect(&(this->dialog_ui->mapnode), SIGNAL(demostration_ready_signal(QString)), this, SLOT(demostration_ready_slot()));
    connect(&(this->dialog_ui->mapnode), SIGNAL(demostration_ready_signal(QString)),this->dock_ui, SLOT(demostration_ready_slot(QString)));
    connect(&(this->dialog_ui->mapnode), SIGNAL(ros_shutdown()), this, SLOT(close()));

    system("gnome-terminal -x bash -c 'bash ~/bash/odom.sh'");
    //std::cout << "567" << std::endl;
}

void MainWindow::closeEvent(QCloseEvent *e){
   system("gnome-terminal -x bash -c 'bash ~/bash/odom_shut.sh'");
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_radioButton_toggled(bool state){
    if(state){
        system("gnome-terminal -x bash -c 'bash ~/bash/laser_2d.sh'");
    }
    else{
        system("gnome-terminal -x bash -c 'bash ~/bash/laser_2d_shut.sh'");
    }
}

void MainWindow::on_radioButton_2_toggled(bool state){
    if(state){
        system("gnome-terminal -x bash -c 'bash ~/bash/record.sh'");
    }
    else{
        system("gnome-terminal -x bash -c 'bash ~/bash/record_shut.sh'");
    }
}


void MainWindow::on_radioButton_3_toggled(bool state){
    if(state){
        if(!ui->radioButton->isChecked()){
            infoLabel->setStyleSheet("color:red");
            infoLabel->setText("需要先启动激光雷达!");
            ui->radioButton_3->setChecked(false);
        }
        else{
            infoLabel->setStyleSheet("color:green");
            infoLabel->setText("正常");
            system("gnome-terminal -x bash -c 'bash ~/bash/mapping_2d.sh'");
        }
    }
    else{

        system("gnome-terminal -x bash -c 'bash ~/bash/mapping_2d_shut.sh'");
    }
}

void MainWindow::on_radioButton_4_toggled(bool state){
    if(state){
        if(!ui->radioButton->isChecked()){
            infoLabel->setStyleSheet("color:red");
            infoLabel->setText("需要先启动激光雷达!");
            ui->radioButton_4->setChecked(false);
        }
        else{
            infoLabel->setStyleSheet("color:green");
            infoLabel->setText("正常");
            system("gnome-terminal -x bash -c 'bash ~/bash/nav_2d.sh'");

            manager_->setFixedFrame("/map");

            rviz::Display *map=manager_->createDisplay("rviz/Map","adjustable map",true);
            ROS_ASSERT(map!=NULL);
            map->subProp("Topic")->setValue("/map");

            rviz::Display *robot=manager_->createDisplay("rviz/RobotModel","adjustable robot",true);
            ROS_ASSERT(robot!=NULL);
            robot->subProp("Robot Description")->setValue("robot_description");

            rviz::Display *laser=manager_->createDisplay("rviz/LaserScan","adjustable laser",true);
            ROS_ASSERT(laser!=NULL);
            laser->subProp("Topic")->setValue("/scan");
            laser->subProp("Size (m)")->setValue("0.1");
            laser->subProp("Color Transformer")->setValue("AxisColor");

            rviz::Display *linear=manager_->createDisplay("rviz/Marker","adjustable linear",true);
            ROS_ASSERT(linear!=NULL);
            linear->subProp("Marker Topic")->setValue("/lmarker");


            rviz::Display *angular=manager_->createDisplay("rviz/MarkerArray","adjustable angular",true);
            ROS_ASSERT(angular!=NULL);
            angular->subProp("Marker Topic")->setValue("/amarker");

            rviz::Display *text=manager_->createDisplay("rviz/Marker","adjustable text",true);
            ROS_ASSERT(text!=NULL);
            text->subProp("Marker Topic")->setValue("/tmarker");

            rviz::Display *goal_marker=manager_->createDisplay("rviz/MarkerArray","adjustable goal",true);
            ROS_ASSERT(goal_marker!=NULL);
            goal_marker->subProp("Marker Topic")->setValue("/goalmarker");


            rviz::Display *goalid_marker=manager_->createDisplay("rviz/MarkerArray","adjustable goalid",true);
            ROS_ASSERT(goalid_marker!=NULL);
            goalid_marker->subProp("Marker Topic")->setValue("/goalid_marker");

            rviz::Display *forbidden=manager_->createDisplay("rviz/MarkerArray","adjustable forbidden",true);
            ROS_ASSERT(forbidden!=NULL);
            forbidden->subProp("Marker Topic")->setValue("/forbidden_marker");

            rviz::Display *id_marker=manager_->createDisplay("rviz/MarkerArray","adjustable id",true);
            ROS_ASSERT(id_marker!=NULL);
            id_marker->subProp("Marker Topic")->setValue("/id_marker");

            rviz::Display *semantic_marker=manager_->createDisplay("rviz/MarkerArray","adjustable semantic",true);
            ROS_ASSERT(semantic_marker!=NULL);
            semantic_marker->subProp("Marker Topic")->setValue("/semantic_marker");

            //rviz::Display *path=manager_->createDisplay("rviz/Path","adjustable",true);
            //ROS_ASSERT(path!=NULL);
            //path->subProp("Topic")->setValue("/move_base/TebLocalPlannerROS/local_plan");
            //path->subProp("Line Style")->setValue("Lines");
            //path->subProp("Pose Style")->setValue("None");
            //path->subProp("Color")->setValue(QColor(125,125,125));
            //path->subProp("Color")->setValue("125; 125; 125");



            manager_->startUpdate();
        }
    }
    else{
        manager_->removeAllDisplays();
        system("gnome-terminal -x bash -c 'bash ~/bash/nav_2d_shut.sh'");
    }
}

void MainWindow::on_radioButton_5_toggled(bool state){
    if(state){
        system("gnome-terminal -x bash -c 'bash ~/bash/remote.sh'");
    }
    else{
        system("gnome-terminal -x bash -c 'bash ~/bash/remote_shut.sh'");
    }
}

void MainWindow::on_radioButton_6_toggled(bool state){
    if(state){
        system("gnome-terminal -x bash -c 'bash ~/bash/laser_3d.sh'");
    }
    else{
        system("gnome-terminal -x bash -c 'bash ~/bash/laser_3d_shut.sh'");
    }
}

void MainWindow::on_radioButton_7_toggled(bool state){
    if(state){
        if(!ui->radioButton_6->isChecked()){
            infoLabel->setStyleSheet("color:red");
            infoLabel->setText("需要先启动激光雷达!");
            ui->radioButton_7->setChecked(false);
        }
        else{
            infoLabel->setStyleSheet("color:green");
            infoLabel->setText("正常");
            system("gnome-terminal -x bash -c 'bash ~/bash/nav_3d.sh'");

            manager_->setFixedFrame("/map");

            rviz::Display *map=manager_->createDisplay("rviz/Map","adjustable map",true);
            ROS_ASSERT(map!=NULL);
            map->subProp("Topic")->setValue("/map");

            rviz::Display *robot=manager_->createDisplay("rviz/RobotModel","adjustable robot",true);
            ROS_ASSERT(robot!=NULL);
            robot->subProp("Robot Description")->setValue("robot_description");

            rviz::Display *laser=manager_->createDisplay("rviz/LaserScan","adjustable laser",true);
            ROS_ASSERT(laser!=NULL);
            laser->subProp("Topic")->setValue("/scan");
            laser->subProp("Size (m)")->setValue("0.1");
            laser->subProp("Color Transformer")->setValue("AxisColor");


            rviz::Display *linear=manager_->createDisplay("rviz/Marker","adjustable linear",true);
            ROS_ASSERT(linear!=NULL);
            linear->subProp("Marker Topic")->setValue("/lmarker");


            rviz::Display *angular=manager_->createDisplay("rviz/MarkerArray","adjustable angular",true);
            ROS_ASSERT(angular!=NULL);
            angular->subProp("Marker Topic")->setValue("/amarker");

            rviz::Display *text=manager_->createDisplay("rviz/Marker","adjustable text",true);
            ROS_ASSERT(text!=NULL);
            text->subProp("Marker Topic")->setValue("/tmarker");

            rviz::Display *goal_marker=manager_->createDisplay("rviz/MarkerArray","adjustable goal",true);
            ROS_ASSERT(goal_marker!=NULL);
            goal_marker->subProp("Marker Topic")->setValue("/goalmarker");


            rviz::Display *goalid_marker=manager_->createDisplay("rviz/MarkerArray","adjustable goalid",true);
            ROS_ASSERT(goalid_marker!=NULL);
            goalid_marker->subProp("Marker Topic")->setValue("/goalid_marker");

            rviz::Display *forbidden=manager_->createDisplay("rviz/MarkerArray","adjustable forbidden",true);
            ROS_ASSERT(forbidden!=NULL);
            forbidden->subProp("Marker Topic")->setValue("/forbidden_marker");

            rviz::Display *id_marker=manager_->createDisplay("rviz/MarkerArray","adjustable id",true);
            ROS_ASSERT(id_marker!=NULL);
            id_marker->subProp("Marker Topic")->setValue("/id_marker");

            rviz::Display *semantic_marker=manager_->createDisplay("rviz/MarkerArray","adjustable semantic",true);
            ROS_ASSERT(semantic_marker!=NULL);
            semantic_marker->subProp("Marker Topic")->setValue("/semantic_marker");

            manager_->startUpdate();
        }
    }
    else{
        manager_->removeAllDisplays();
        system("gnome-terminal -x bash -c 'bash ~/bash/nav_3d_shut.sh'");
    }
}

void MainWindow::on_radioButton_8_toggled(bool state){
    if(state){
        if(!ui->radioButton_6->isChecked()){
            infoLabel->setStyleSheet("color:red");
            infoLabel->setText("需要先启动激光雷达!");
            ui->radioButton_8->setChecked(false);
        }
        else{
            infoLabel->setStyleSheet("color:green");
            infoLabel->setText("正常");
            system("gnome-terminal -x bash -c 'bash ~/bash/mapping_3d.sh'");
        }
    }
    else{
        system("gnome-terminal -x bash -c 'bash ~/bash/mapping_3d_shut.sh'");
    }
}

void MainWindow::on_radioButton_9_toggled(bool state){
    if(state){

        rviz::Display *path_marker=manager_->createDisplay("rviz/MarkerArray","adjustable path",true);
        ROS_ASSERT(path_marker!=NULL);
        path_marker->subProp("Marker Topic")->setValue("/path_marker");
        manager_->startUpdate();
        emit record_path_signal();
    }
    else{
        emit save_path_signal();
    }
}

void MainWindow::on_radioButton_10_toggled(bool state){
    if(state){
        emit track_signal();
    }
    else{
        emit track_shut_signal();
    }
}

void MainWindow::on_radioButton_11_toggled(bool state){
    if(state){
        rviz::Display *path_marker=manager_->createDisplay("rviz/MarkerArray","adjustable path",true);
        ROS_ASSERT(path_marker!=NULL);
        path_marker->subProp("Marker Topic")->setValue("/path_marker");

        rviz::Display *route_marker=manager_->createDisplay("rviz/MarkerArray","adjustable route",true);
        ROS_ASSERT(route_marker!=NULL);
        route_marker->subProp("Marker Topic")->setValue("/route_marker");

        manager_->startUpdate();
        emit track_2d_signal();
        system("gnome-terminal -x bash -c 'bash ~/bash/pursuit.sh'");
    }
    else{
        system("gnome-terminal -x bash -c 'bash ~/bash/pursuit_shut.sh'");
    }
}

void MainWindow::on_pushButton_clicked(){
    QString filename = QFileDialog::getOpenFileName(this, "select map", "/","PCD Files(*.pcd);;All Files(*)");
    ui->lineEdit->setText(filename);
}

void MainWindow::on_pushButton_2_clicked(){
    std::string home_path =  std::getenv("HOME");
    std::string filename = home_path + "/bash/param.yaml";
    int mappath_linenum = 51;
    ModifyLineData(filename, mappath_linenum, "  mapPath: " + ui->lineEdit->text().toStdString());
}

void MainWindow::on_pushButton_3_clicked(){
    this->demo_ui->show();
    //

}

void MainWindow::on_pushButton_4_clicked(){
    this->nav_ui->show();
    //

}

void MainWindow::on_pushButton_23_clicked(){
    this->hide();
    arm_ui->move(this->x(), this->y());
    connect(arm_ui, SIGNAL(sendsignal()), this, SLOT(reshow()));
    arm_ui->show();
}


void MainWindow::on_pushButton_24_clicked(){
    current_tool= tool_manager_->addTool("rviz/SetInitialPose");
    //设置当前使用的工具为SetInitialPose（实现在地图上标点）
    tool_manager_->setCurrentTool( current_tool );
    manager_->startUpdate();
}

void MainWindow::on_pushButton_25_clicked(){
    //添加工具
    current_tool= tool_manager_->addTool("rviz/SetGoal");
    //设置goal的话题
    rviz::Property* pro= current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("coarse_goal");//move_base_simple/goal
    //设置当前frame
    manager_->setFixedFrame("map");
    //设置当前使用的工具为SetGoal（实现在地图上标点）
    tool_manager_->setCurrentTool( current_tool );

    manager_->startUpdate();
}

void MainWindow::on_pushButton_26_clicked(){
    dialog_ui->show();
}

void MainWindow::on_pushButton_27_clicked(){
    delete_ui->show();
}

void MainWindow::on_pushButton_28_clicked(){
    semantic_ui->show();
}

void MainWindow::on_pushButton_29_clicked(){
    multigoal_ui->show();
}


void MainWindow::on_pushButton_31_clicked(){
    this->dock_ui->show();
    //emit dock_signal();
}

void MainWindow::reshow(){
    this->show();
}

void MainWindow::power_slot(float p){
    ui->progressBar->setValue(p);
    if(p<=20)
        ui->progressBar->setStyleSheet("QProgressBar::chunk { background-color: rgb(255, 0, 0) }");
    else
        ui->progressBar->setStyleSheet("QProgressBar::chunk { background-color: rgb(0, 255, 0) }");
}

void MainWindow::flag_slot(float f){
    if(f == 0)
        ui->label_2->setVisible(false);
    else
        ui->label_2->setVisible(true);
}

void MainWindow::temp_slot(float t){
    ui->label_4->setText(QString::number(t, 10, 1)+"°C");
}

void MainWindow::startp_slot(){
    current_tool=tool_manager_->addTool("rviz/PublishPoint");
    tool_manager_->setCurrentTool( current_tool );

    rviz::Property* pro= current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("/startp");

    manager_->setFixedFrame("map");
    manager_->startUpdate();
}

void MainWindow::endp_slot(){
    current_tool=tool_manager_->addTool("rviz/PublishPoint");
    tool_manager_->setCurrentTool( current_tool );

    rviz::Property* pro= current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("/endp");

    manager_->setFixedFrame("map");
    manager_->startUpdate();
}

void MainWindow::markersignal_slot(){
    dialog_ui->hide();
}

void MainWindow::line_startp_slot(){
    current_tool=tool_manager_->addTool("rviz/PublishPoint");
    tool_manager_->setCurrentTool( current_tool );

    rviz::Property* pro= current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("/line_startp");

    manager_->setFixedFrame("map");
    manager_->startUpdate();
}

void MainWindow::line_endp_slot(){
    current_tool=tool_manager_->addTool("rviz/PublishPoint");
    tool_manager_->setCurrentTool( current_tool );

    rviz::Property* pro= current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("/line_endp");

    manager_->setFixedFrame("map");
    manager_->startUpdate();
}

void MainWindow::line_markersignal_slot(){
    dialog_ui->hide();
}

void MainWindow::pointp_slot(){
    current_tool=tool_manager_->addTool("rviz/PublishPoint");
    tool_manager_->setCurrentTool( current_tool );

    rviz::Property* pro= current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("/pointp");

    manager_->setFixedFrame("map");
    manager_->startUpdate();
}


void MainWindow::point_markersignal_slot(){
    dialog_ui->hide();
}

void MainWindow::semanticp_slot(){
    current_tool=tool_manager_->addTool("rviz/SetGoal");
    tool_manager_->setCurrentTool( current_tool );

    rviz::Property* pro= current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("/semanticp");

    manager_->setFixedFrame("map");
    manager_->startUpdate();
}

void MainWindow::semantic_markersignal_slot(){
    semantic_ui->hide();
}

void MainWindow::setgoal_slot(){
    //添加工具
    current_tool= tool_manager_->addTool("rviz/SetGoal");
    //设置goal的话题
    rviz::Property* pro= current_tool->getPropertyContainer();
    pro->subProp("Topic")->setValue("multigoal");
    //设置当前frame
    manager_->setFixedFrame("map");
    //设置当前使用的工具为SetGoal（实现在地图上标点）
    tool_manager_->setCurrentTool( current_tool );

    manager_->startUpdate();
}

/************************************
@ Brief:    修改行数据
@ Author: woniu201
@ Created: 2018/08/31
@ Return:
************************************/
void MainWindow::ModifyLineData(std::string fileName, int lineNum, std::string lineData)
{
    std::ifstream in;
    in.open(fileName);
    std::string strFileData = "";
    int line = 1;
    char tmpLineData[1024] = {0};
    while(in.getline(tmpLineData, sizeof(tmpLineData)))
    {
        if (line == lineNum)
        {
        strFileData += lineData;
        strFileData += "\n";
        }
    else
        {
        strFileData += CharToStr(tmpLineData);
        strFileData += "\n";
        }
    line++;
    }
    in.close();
    //写入文件
    std::ofstream out;
    out.open(fileName);
    out.flush();
    out<<strFileData;
    out.close();
}

std::string MainWindow::CharToStr(char * contentChar)
{
    std::string tempStr;
    for (int i=0;contentChar[i]!='\0';i++)
    {
        tempStr+=contentChar[i];
    }
    return tempStr;
}


void MainWindow::dock_ready_slot(){
    system("gnome-terminal -x bash -c 'bash ~/bash/icp.sh'");
}

void MainWindow::dock_ready_slot2(){
    sleep(5);
    system("gnome-terminal -x bash -c 'bash ~/bash/dock.sh'");
    return;
}

void MainWindow::demostration_ready_slot(){
    sleep(3);
    system("gnome-terminal -x bash -c 'bash ~/bash/demostration_shut.sh'");
    //ROS_INFO("DEMO READY");
    QMessageBox::information(NULL, "消息", "示教完成!", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
}


