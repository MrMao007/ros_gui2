/********************************************************************************
** Form generated from reading UI file 'dock.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DOCK_H
#define UI_DOCK_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Dock
{
public:
    QPushButton *pushButton;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLabel *label;
    QComboBox *comboBox;

    void setupUi(QDialog *Dock)
    {
        if (Dock->objectName().isEmpty())
            Dock->setObjectName(QStringLiteral("Dock"));
        Dock->resize(299, 139);
        pushButton = new QPushButton(Dock);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(190, 100, 89, 25));
        gridLayoutWidget = new QWidget(Dock);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(30, 30, 251, 31));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        comboBox = new QComboBox(gridLayoutWidget);
        comboBox->setObjectName(QStringLiteral("comboBox"));

        gridLayout->addWidget(comboBox, 0, 1, 1, 1);

        gridLayout->setColumnStretch(0, 1);
        gridLayout->setColumnStretch(1, 2);

        retranslateUi(Dock);

        QMetaObject::connectSlotsByName(Dock);
    } // setupUi

    void retranslateUi(QDialog *Dock)
    {
        Dock->setWindowTitle(QApplication::translate("Dock", "\345\210\240\351\231\244\347\246\201\350\241\214\345\214\272\345\237\237", 0));
        pushButton->setText(QApplication::translate("Dock", "\347\241\256\345\256\232", 0));
        label->setText(QApplication::translate("Dock", "\345\257\271\346\216\245\347\202\271\357\274\232", 0));
    } // retranslateUi

};

namespace Ui {
    class Dock: public Ui_Dock {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DOCK_H
