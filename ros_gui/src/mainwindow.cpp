#include "../include/ros_gui/mainwindow.h"
#include "../build/ui_mainwindow.h"
#include <math.h>

#define pi 3.1415926


MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    markernode(argc, argv)
{
    ui->setupUi(this);
    this->image = new QImage();
    double x1=-50.0,y1=0.0,w1=20.0,h1=20.0;
    double w2=2*w1,h2=h1/4.0;
    double x2=x1+w1/2.0;
    double y2=y1+h1/2.0-h2/2.0;
    double x3_1=x2+w2,y3_1=y2-h2;
    double x3_2=x2+w2,y3_2=y2+h2+h2;
    double x3_3=x2+w2+2*h2,y3_3=(y3_1+y3_2)/2;
    QPolygonF polygon;
    temp  = new QGraphicsPolygonItem();
    //polygon<<QPointF(-nwith/2+5, nheight/2-10-45)<<QPointF(-nwith/2+5+30, nheight/2-10-30)<<QPointF(-nwith/2+5+30, nheight/2-10-60);
    polygon<<QPointF(x3_1,y3_1)<<QPointF(x3_2,y3_2)<<QPointF(x3_3,y3_3);
    temp->setPolygon(polygon);
    temp->setPen(QPen(QColor(255,0,0)));
    temp->setBrush(QBrush(QColor(255,0,0)));


    this->pEllipseItemTmp = new QGraphicsEllipseItem(QRectF(x1, y1, w1, h1));
    pEllipseItemTmp->setPen(QPen(QColor(255,0,0)));
    pEllipseItemTmp->setBrush(QBrush(QColor(255,0,0)));
    this->direction = new QGraphicsRectItem(QRectF(x2,y2,w2,h2));
    direction->setPen(QPen(QColor(255,0,0)));
    direction->setBrush(QBrush(QColor(255,0,0)));

   /* this->itemg = new QGraphicsItemGroup();
    this->itemg->addToGroup(this->pEllipseItemTmp);
    this->itemg->addToGroup(this->direction);
    this->itemg->addToGroup(temp);*/
    this->direction->setParentItem(pEllipseItemTmp);
    temp->setParentItem(pEllipseItemTmp);

    this->left = creatLeft();
    this->right = creatRight();
    this->up = creatUp();
    this->down = creatDown();
    this->statusLabel = new QLabel;

    this->arm_ui = new arm(argc, argv);
    this->dialog_ui = new Dialog(argc, argv);
    this->delete_ui = new Delete();
    this->semantic_ui = new Semantic();
    this->multigoal_ui = new Multigoal(argc, argv);

    statusLabel->setMinimumSize(100,20); //设置标签最小尺寸
    statusLabel->setFrameShape(QFrame::WinPanel); //设置标签形状
    statusLabel->setFrameShadow(QFrame::Sunken); //设置标签阴影
    statusBar()->addWidget(statusLabel);
    statusLabel->setStyleSheet("color:gray");
    statusLabel->setText("遥控停止...");

    this->navLabel = new QLabel;
    navLabel->setMinimumSize(100,20); //设置标签最小尺寸
    navLabel->setFrameShape(QFrame::WinPanel); //设置标签形状
    navLabel->setFrameShadow(QFrame::Sunken); //设置标签阴影
    statusBar()->addWidget(navLabel);
    navLabel->setStyleSheet("color:gray");
    navLabel->setText("导航停止...");

    this->laser2Label = new QLabel;
    laser2Label->setMinimumSize(100,20); //设置标签最小尺寸
    laser2Label->setFrameShape(QFrame::WinPanel); //设置标签形状
    laser2Label->setFrameShadow(QFrame::Sunken); //设置标签阴影
    statusBar()->addWidget(laser2Label);
    laser2Label->setStyleSheet("color:red");
    laser2Label->setText("2D激光停止...");

    this->laser3Label = new QLabel;
    laser3Label->setMinimumSize(100,20); //设置标签最小尺寸
    laser3Label->setFrameShape(QFrame::WinPanel); //设置标签形状
    laser3Label->setFrameShadow(QFrame::Sunken); //设置标签阴影
    statusBar()->addWidget(laser3Label);
    laser3Label->setStyleSheet("color:red");
    laser3Label->setText("3D激光停止...");

    this->odomLabel = new QLabel;
    odomLabel->setMinimumSize(100,20); //设置标签最小尺寸
    odomLabel->setFrameShape(QFrame::WinPanel); //设置标签形状
    odomLabel->setFrameShadow(QFrame::Sunken); //设置标签阴影
    statusBar()->addWidget(odomLabel);
    odomLabel->setStyleSheet("color:red");
    odomLabel->setText("里程计停止...");

    this->mapping2Label = new QLabel;
    mapping2Label->setMinimumSize(100,20); //设置标签最小尺寸
    mapping2Label->setFrameShape(QFrame::WinPanel); //设置标签形状
    mapping2Label->setFrameShadow(QFrame::Sunken); //设置标签阴影
    statusBar()->addWidget(mapping2Label);
    mapping2Label->setStyleSheet("color:gray");
    mapping2Label->setText("2D建图停止...");

    this->mapping3Label = new QLabel;
    mapping3Label->setMinimumSize(100,20); //设置标签最小尺寸
    mapping3Label->setFrameShape(QFrame::WinPanel); //设置标签形状
    mapping3Label->setFrameShadow(QFrame::Sunken); //设置标签阴影
    statusBar()->addWidget(mapping3Label);
    mapping3Label->setStyleSheet("color:gray");
    mapping3Label->setText("3D建图停止...");


    this->loc3Label = new QLabel;
    loc3Label->setMinimumSize(100,20); //设置标签最小尺寸
    loc3Label->setFrameShape(QFrame::WinPanel); //设置标签形状
    loc3Label->setFrameShadow(QFrame::Sunken); //设置标签阴影
    statusBar()->addWidget(loc3Label);
    loc3Label->setStyleSheet("color:gray");
    loc3Label->setText("3D定位停止...");

    markernode.init();

    ros::init(argc,argv,"mainwindow",ros::init_options::AnonymousName);
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


    connect(&markernode, SIGNAL(power(float)), this, SLOT(power_slot(float)));
    connect(&markernode, SIGNAL(power_flag(float)), this, SLOT(flag_slot(float)));
    connect(&markernode, SIGNAL(temp(float)), this, SLOT(temp_slot(float)));
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
}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::on_pushButton_2_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/remote.sh'");
    this->statusLabel->setStyleSheet("color:green");
    this->statusLabel->setText("遥控中...");
}

void MainWindow::on_pushButton_3_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/remote_shut.sh'");
    this->statusLabel->setStyleSheet("color:gray");
    this->statusLabel->setText("遥控停止...");
}

void MainWindow::on_pushButton_4_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/laser_2d.sh'");
    this->laser2Label->setStyleSheet("color:green");
    this->laser2Label->setText("2D激光已启动！");
}

void MainWindow::on_pushButton_5_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/odom.sh'");
    this->odomLabel->setStyleSheet("color:green");
    this->odomLabel->setText("里程计已启动！");
}

void MainWindow::on_pushButton_6_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/mapping_2d.sh'");
    this->mapping2Label->setStyleSheet("color:green");
    this->mapping2Label->setText("2D建图中...");
}

void MainWindow::on_pushButton_7_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/laser_2d_shut.sh'");
    laser2Label->setStyleSheet("color:red");
    laser2Label->setText("2D激光停止...");
}

void MainWindow::on_pushButton_8_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/odom_shut.sh'");
    odomLabel->setStyleSheet("color:red");
    odomLabel->setText("里程计停止...");
}

void MainWindow::on_pushButton_9_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/save_map.sh'");

}

void MainWindow::on_pushButton_10_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/mapping_2d_shut.sh'");
    mapping2Label->setStyleSheet("color:gray");
    mapping2Label->setText("2D建图停止...");
}



void MainWindow::on_pushButton_12_clicked()
{
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
    laser->subProp("Topic")->setValue("rightlaser/scan");
    laser->subProp("Size (m)")->setValue("0.1");

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
    this->navLabel->setStyleSheet("color:green");
    this->navLabel->setText("自主导航中...");
}



void MainWindow::on_pushButton_14_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/nav_2d_shut.sh'");
    this->navLabel->setStyleSheet("color:gray");
    this->navLabel->setText("导航停止中...");
}

void MainWindow::on_pushButton_15_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/mapping_3d.sh'");
    mapping3Label->setStyleSheet("color:green");
    mapping3Label->setText("3D建图中...");
}

void MainWindow::on_pushButton_16_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/loc_3d.sh'");
    loc3Label->setStyleSheet("color:green");
    loc3Label->setText("3D定位中...");
}

void MainWindow::on_pushButton_17_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/loc_3d_shut.sh'");
    loc3Label->setStyleSheet("color:gray");
    loc3Label->setText("3D定位停止...");
}

void MainWindow::on_pushButton_18_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/mapping_3d_shut.sh'");
    mapping3Label->setStyleSheet("color:gray");
    mapping3Label->setText("3D建图停止...");
}


void MainWindow::on_pushButton_21_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/laser_3d.sh'");
    laser3Label->setStyleSheet("color:green");
    laser3Label->setText("3D激光已启动！");
}

void MainWindow::on_pushButton_22_clicked()
{
    system("gnome-terminal -x bash -c 'bash ~/bash/laser_3d_shut.sh'");
    laser3Label->setStyleSheet("color:red");
    laser3Label->setText("3D激光停止...");
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
    pro->subProp("Topic")->setValue("/move_base_simple/goal");
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

void MainWindow::on_pushButton_30_clicked(){
    //manager_->setFixedFrame("/base_footprint");
    system("bash ~/bash/test.sh &");
}

void MainWindow::on_pushButton_31_clicked(){
    //manager_->setFixedFrame("/map");
    system("bash ~/bash/test_shut.sh &");
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
    current_tool=tool_manager_->addTool("rviz/PublishPoint");
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
/*
void MainWindow::recvShowPicSignal(QImage image)
{
    QPixmap ConvertPixmap=QPixmap::fromImage(image);//The QPixmap class is an off-screen image representation that can be used as a paint device
    QGraphicsScene  *qgraphicsScene = new QGraphicsScene;//要用QGraphicsView就必须要有QGraphicsScene搭配着用
    m_Image = new ImageWidget(&ConvertPixmap);//实例化类ImageWidget的对象m_Image，该类继承自QGraphicsItem，是自己写的类
    int nwith = ui->ImageGraphic->width();//获取界面控件Graphics View的宽度
    int nheight = ui->ImageGraphic->height();//获取界面控件Graphics View的高度
    m_Image->setQGraphicsViewWH(nwith,nheight);
    //将界面控件Graphics View的width和height传进类m_Image中
    pEllipseItemTmp->setParentItem(m_Image);
    qgraphicsScene->addItem(m_Image);//将QGraphicsItem类对象放进QGraphicsScene中
    //qgraphicsScene->addItem(itemg);
    qgraphicsScene->addItem(left);
    qgraphicsScene->addItem(right);
    qgraphicsScene->addItem(up);
    qgraphicsScene->addItem(down);
    ui->ImageGraphic->setSceneRect(QRectF(-(nwith/2),-(nheight/2),nwith,nheight));//使视窗的大小固定在原始大小，不会随图片的放大而放大（默认状态下图片放大的时候视窗两边会自动出现滚动条，并且视窗内的视野会变大），防止图片放大后重新缩小的时候视窗太大而不方便观察图片
    ui->ImageGraphic->setScene(qgraphicsScene);//Sets the current scene to scene. If scene is already being viewed, this function does nothing.
    ui->ImageGraphic->setFocus();//将界面的焦点设置到当前Graphics View控件
}

void MainWindow::keyPressEvent(QKeyEvent *e)//实例一个键盘事件的对象e
{
    /*QPointF p = pEllipseItemTmp->mapFromParent(pEllipseItemTmp->pos());
    double x = p.x();//获取按钮x坐标
    double y = p.y();//获取按钮y坐标
    QTransform transform;
    double pace =5.0;
    if(cur_angle>=360)
        cur_angle=cur_angle-360;
    switch (e->key())//选择判断语句(判断是否有键盘按钮事件)
    {
    case Qt::Key_W :pEllipseItemTmp->moveBy(pace*cos(cur_angle/180*pi),pace*sin(cur_angle/180*pi));
                    up->setBrush(QBrush(QColor(255,255,0))); break;//移动按钮y坐标
    case Qt::Key_S :pEllipseItemTmp->moveBy(-pace*cos(cur_angle/180*pi),-pace*sin(cur_angle/180*pi));
                    down->setBrush(QBrush(QColor(255,255,0))); break;
    case Qt::Key_A :cur_angle -=15;
                    pEllipseItemTmp->setTransformOriginPoint(-40,10);
                    pEllipseItemTmp->setRotation(cur_angle);
                    left->setBrush(QBrush(QColor(255,255,0))); break;//移动按钮x坐标
    case Qt::Key_D :cur_angle +=15;
                    pEllipseItemTmp->setTransformOriginPoint(-40,10);
                    pEllipseItemTmp->setRotation(cur_angle);
                    right->setBrush(QBrush(QColor(255,255,0))); break;
    default:
        break;
    }
        //std::cout << pEllipseItemTmp->scenePos().x() << " "<< pEllipseItemTmp->scenePos().y()<< std::endl;
}

void MainWindow::keyReleaseEvent(QKeyEvent *e){
    switch (e->key())//选择判断语句(判断是否有键盘按钮事件)
    {
    case Qt::Key_W :up->setBrush(QBrush(QColor(255,255,255))); break;//移动按钮y坐标
    case Qt::Key_S :down->setBrush(QBrush(QColor(255,255,255))); break;
    case Qt::Key_A :left->setBrush(QBrush(QColor(255,255,255))); break;//移动按钮x坐标
    case Qt::Key_D :right->setBrush(QBrush(QColor(255,255,255))); break;
    default:
        break;
    }
}
*/

QGraphicsPolygonItem* MainWindow::creatLeft(){
    QPolygonF polygon;
    QGraphicsPolygonItem* upItem  = new QGraphicsPolygonItem();
    //polygon<<QPointF(-nwith/2+5, nheight/2-10-45)<<QPointF(-nwith/2+5+30, nheight/2-10-30)<<QPointF(-nwith/2+5+30, nheight/2-10-60);
    polygon<<QPointF(-500+5, 330-10-45)<<QPointF(-500+5+30, 330-10-30)<<QPointF(-500+5+30, 330-10-60);
    upItem->setPolygon(polygon);
    upItem->setPen(QPen(QColor(255,0,0)));
    upItem->setBrush(QBrush(QColor(255,255,255)));
    return upItem;
}


QGraphicsPolygonItem* MainWindow::creatRight(){
    QPolygonF polygon;
    QGraphicsPolygonItem* upItem  = new QGraphicsPolygonItem();
    //polygon<<QPointF(-nwith/2+5, nheight/2-10-45)<<QPointF(-nwith/2+5+30, nheight/2-10-30)<<QPointF(-nwith/2+5+30, nheight/2-10-60);
    polygon<<QPointF(-500+15+90, 330-10-45)<<QPointF(-500+15+60, 330-10-30)<<QPointF(-500+15+60, 330-10-60);
    upItem->setPolygon(polygon);
    upItem->setPen(QPen(QColor(255,0,0)));
    upItem->setBrush(QBrush(QColor(255,255,255)));
    return upItem;
}

QGraphicsPolygonItem* MainWindow::creatUp(){
    QPolygonF polygon;
    QGraphicsPolygonItem* upItem  = new QGraphicsPolygonItem();
    //polygon<<QPointF(-nwith/2+5, nheight/2-10-45)<<QPointF(-nwith/2+5+30, nheight/2-10-30)<<QPointF(-nwith/2+5+30, nheight/2-10-60);
    polygon<<QPointF(-500+10+30, 330-15-60)<<QPointF(-500+10+60, 330-15-60)<<QPointF(-500+10+45, 330-15-90);
    upItem->setPolygon(polygon);
    upItem->setPen(QPen(QColor(255,0,0)));
    upItem->setBrush(QBrush(QColor(255,255,255)));
    return upItem;
}

QGraphicsPolygonItem* MainWindow::creatDown(){
    QPolygonF polygon;
    QGraphicsPolygonItem* upItem  = new QGraphicsPolygonItem();
    //polygon<<QPointF(-nwith/2+5, nheight/2-10-45)<<QPointF(-nwith/2+5+30, nheight/2-10-30)<<QPointF(-nwith/2+5+30, nheight/2-10-60);
    polygon<<QPointF(-500+10+30, 330-5-30)<<QPointF(-500+10+60, 330-5-30)<<QPointF(-500+10+45, 330-5);
    upItem->setPolygon(polygon);
    upItem->setPen(QPen(QColor(255,0,0)));
    upItem->setBrush(QBrush(QColor(255,255,255)));
    return upItem;
}
