#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <vector>
#include "mapNode.h"
#include "delete.h"

namespace Ui {
class Dialog;
}

class Dialog : public QDialog
{
    Q_OBJECT

public:
    explicit Dialog(int argc, char** argv,QWidget *parent = 0);
    ~Dialog();

     Ui::MapNode mapnode;
signals:
    void startp();
    void endp();
    void markersignal();

    void line_startp();
    void line_endp();
    void line_markersignal();

    void pointp();
    void point_markersignal(double radius);

public Q_SLOTS:
    void startpUpdated_slot(double x,double y);

    void endpUpdated_slot(double x,double y);

    void line_startpUpdated_slot(double x,double y);

    void line_endpUpdated_slot(double x,double y);

    void pointpUpdated_slot(double x,double y);

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_13_clicked();

    void on_pushButton_14_clicked();

    void on_pushButton_15_clicked();

    void on_pushButton_16_clicked();

    void on_pushButton_20_clicked();

    void on_pushButton_17_clicked();

    void on_pushButton_18_clicked();

private:
    Ui::Dialog *ui;

    //Ui::Delete *delete_ui;
};

#endif // DIALOG_H
