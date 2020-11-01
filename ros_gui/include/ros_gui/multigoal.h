#ifndef MULTIGOAL_H
#define MULTIGOAL_H

#include <QDialog>
#include <vector>
#include <QString>
//#include "multinode.h"

namespace Ui {
class Multigoal;
}

class Multigoal : public QDialog
{
    Q_OBJECT

public:
    explicit Multigoal(int argc, char** argv, QWidget *parent = 0);
    ~Multigoal();

    //Ui::Multinode multinode;

signals:
    void setgoal_signal();

    void multigoal_signal();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::Multigoal *ui;

};

#endif // MULTIGOAL_H
