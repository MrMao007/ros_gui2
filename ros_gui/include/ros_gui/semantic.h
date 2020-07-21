#ifndef SEMANTIC_H
#define SEMANTIC_H

#include <QDialog>
#include <vector>
#include <QString>

namespace Ui {
class Semantic;
}

class Semantic : public QDialog
{
    Q_OBJECT

public:
    explicit Semantic(QWidget *parent = 0);
    ~Semantic();

signals:
    void semanticp();

    void semantic_signal(std::string);

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::Semantic *ui;

};

#endif // SEMANTIC_H
