#ifndef DELETE_H
#define DELETE_H

#include <QDialog>
#include <vector>
#include <QString>

namespace Ui {
class Delete;
}

class Delete : public QDialog
{
    Q_OBJECT

public:
    explicit Delete(QWidget *parent = 0);
    ~Delete();

signals:

    void delete_signal(int id);

private slots:
    void on_pushButton_clicked();

private:
    Ui::Delete *ui;

};

#endif // DIALOG_H
