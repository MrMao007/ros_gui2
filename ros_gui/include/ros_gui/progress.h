#ifndef PROGRESS_H
#define PROGRESS_H

#include <QDialog>
#include <vector>
#include <QString>

namespace Ui {
class Progress;
}

class Progress : public QDialog
{
    Q_OBJECT

public:
    explicit Progress(QWidget *parent = 0);
    ~Progress();
    void setProgress(float per, QString format);
private:
    Ui::Progress *ui;

};

#endif // SEMANTIC_H
