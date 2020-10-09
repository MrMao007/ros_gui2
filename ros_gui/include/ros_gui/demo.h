#ifndef DEMO_H
#define DEMO_H

#include <QDialog>
#include <vector>
#include <QString>
#include <QMessageBox>

namespace Ui {
class Demo;
}

class Demo : public QDialog
{
    Q_OBJECT

public:
    explicit Demo(QWidget *parent = 0);
    ~Demo();

signals:
    void demostration_signal(QString demo_name);

private slots:
    void on_pushButton_clicked();


private:
    Ui::Demo *ui;

};

#endif // SEMANTIC_H
