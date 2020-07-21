#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "imagewidget.h"
#include <QKeyEvent>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <QFileDialog>
#include <QLabel>
#include "arm.h"
#include "markerNode.h"
#include "dialog.h"
#include "delete.h"
#include "semantic.h"
#include "multigoal.h"



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    //void recvShowPicSignal(QImage image);//接收并显示图片的函数
    //void keyPressEvent(QKeyEvent *);//键盘按下事件
    //void keyReleaseEvent(QKeyEvent *);
    QGraphicsPolygonItem* creatLeft();
    QGraphicsPolygonItem* creatRight();
    QGraphicsPolygonItem* creatUp();
    QGraphicsPolygonItem* creatDown();

public Q_SLOTS:

    void power_slot(float p);

    void flag_slot(float f);

    void temp_slot(float t);

    void startp_slot();

    void endp_slot();

    void markersignal_slot();

    void line_startp_slot();

    void line_endp_slot();

    void line_markersignal_slot();

    void pointp_slot();

    void point_markersignal_slot();

    void semanticp_slot();

    void semantic_markersignal_slot();

    void setgoal_slot();

private slots:


    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_9_clicked();

    void on_pushButton_10_clicked();


    void on_pushButton_12_clicked();



    void on_pushButton_14_clicked();

    void on_pushButton_15_clicked();

    void on_pushButton_16_clicked();

    void on_pushButton_17_clicked();

    void on_pushButton_18_clicked();





    void on_pushButton_21_clicked();

    void on_pushButton_22_clicked();

    void on_pushButton_23_clicked();

    void on_pushButton_24_clicked();

    void on_pushButton_25_clicked();

    void on_pushButton_26_clicked();

    void on_pushButton_27_clicked();

    void on_pushButton_28_clicked();

    void on_pushButton_29_clicked();

    void on_pushButton_30_clicked();

    void on_pushButton_31_clicked();

    void reshow();
private:
    Ui::MainWindow *ui;
    ImageWidget *m_Image;
    QImage *image;
    QGraphicsEllipseItem* pEllipseItemTmp;
    QGraphicsRectItem* direction;
    QGraphicsPolygonItem* temp;
    QGraphicsItemGroup *itemg;
    QGraphicsPolygonItem* left;
    QGraphicsPolygonItem* right;
    QGraphicsPolygonItem* up;
    QGraphicsPolygonItem* down;
    QLabel *statusLabel;
    QLabel *navLabel;
    QLabel *laser2Label;
    QLabel *laser3Label;
    QLabel *odomLabel;
    QLabel *mapping2Label;
    QLabel *mapping3Label;
    QLabel *loc2Label;
    QLabel *loc3Label;
    arm *arm_ui;
    Dialog *dialog_ui;
    Delete *delete_ui;
    Semantic *semantic_ui;
    Multigoal *multigoal_ui;
    double cur_angle = 0;
    rviz::VisualizationManager *manager_;
    rviz::RenderPanel *render_panel_;
    rviz::ViewManager* viewManager_;
    rviz::ToolManager* tool_manager_;
    rviz::Tool *current_tool;
    Ui::MarkerNode markernode;

};

#endif // MAINWINDOW_H