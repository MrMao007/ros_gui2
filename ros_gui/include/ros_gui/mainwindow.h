#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
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
    void ModifyLineData(std::string fileName, int lineNum, std::string lineData);
    std::string CharToStr(char * contentChar);
signals:
    void record_path_signal();

    void save_path_signal();

    void track_signal();

    void track_shut_signal();

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

    void on_radioButton_toggled(bool state);
    void on_radioButton_2_toggled(bool state);
    void on_radioButton_3_toggled(bool state);
    void on_radioButton_4_toggled(bool state);
    void on_radioButton_5_toggled(bool state);
    void on_radioButton_6_toggled(bool state);
    void on_radioButton_7_toggled(bool state);
    void on_radioButton_8_toggled(bool state);
    void on_radioButton_9_toggled(bool state);
    void on_radioButton_10_toggled(bool state);

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

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

    QLabel *statusLabel;
    QLabel *navLabel;
    QLabel *laser2Label;
    QLabel *laser3Label;
    QLabel *odomLabel;
    QLabel *mapping2Label;
    QLabel *mapping3Label;
    QLabel *infoLabel;
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
