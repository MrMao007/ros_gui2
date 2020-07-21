/********************************************************************************
** Form generated from reading UI file 'multigoal.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MULTIGOAL_H
#define UI_MULTIGOAL_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_Multigoal
{
public:
    QPushButton *pushButton;
    QPushButton *pushButton_2;

    void setupUi(QDialog *Multigoal)
    {
        if (Multigoal->objectName().isEmpty())
            Multigoal->setObjectName(QStringLiteral("Multigoal"));
        Multigoal->resize(351, 276);
        pushButton = new QPushButton(Multigoal);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(120, 60, 111, 41));
        pushButton_2 = new QPushButton(Multigoal);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setGeometry(QRect(120, 160, 111, 41));

        retranslateUi(Multigoal);

        QMetaObject::connectSlotsByName(Multigoal);
    } // setupUi

    void retranslateUi(QDialog *Multigoal)
    {
        Multigoal->setWindowTitle(QApplication::translate("Multigoal", "\345\244\232\347\202\271\345\257\274\350\210\252", 0));
        pushButton->setText(QApplication::translate("Multigoal", "\351\200\211\345\217\226\345\257\274\350\210\252\347\202\271", 0));
        pushButton_2->setText(QApplication::translate("Multigoal", "\345\274\200\345\247\213\345\257\274\350\210\252", 0));
    } // retranslateUi

};

namespace Ui {
    class Multigoal: public Ui_Multigoal {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MULTIGOAL_H
