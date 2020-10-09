#ifndef DOCK_H
#define DOCK_H

#include <QDialog>
#include <vector>
#include <QString>
#include <opencv2/opencv.hpp>

namespace Ui {
class Dock;
}

class Dock : public QDialog
{
    Q_OBJECT

public:
    explicit Dock(QWidget *parent = 0);
    std::vector<QString> dock_names;
    ~Dock();

signals:
    void dock_signal(QString dock_n);


private slots:
    void on_pushButton_clicked();

    void demostration_ready_slot(QString demo_n);



private:
    Ui::Dock *ui;

};

#endif // SEMANTIC_H
