/********************************************************************************
** Form generated from reading UI file 'nav.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NAV_H
#define UI_NAV_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Nav
{
public:
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton;

    void setupUi(QDialog *Nav)
    {
        if (Nav->objectName().isEmpty())
            Nav->setObjectName(QStringLiteral("Nav"));
        Nav->resize(350, 165);
        gridLayoutWidget = new QWidget(Nav);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(50, 30, 251, 107));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_2 = new QPushButton(gridLayoutWidget);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        gridLayout->addWidget(pushButton_2, 2, 0, 1, 1);

        pushButton_3 = new QPushButton(gridLayoutWidget);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));

        gridLayout->addWidget(pushButton_3, 3, 0, 1, 1);

        pushButton = new QPushButton(gridLayoutWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        gridLayout->addWidget(pushButton, 1, 0, 1, 1);


        retranslateUi(Nav);

        QMetaObject::connectSlotsByName(Nav);
    } // setupUi

    void retranslateUi(QDialog *Nav)
    {
        Nav->setWindowTitle(QApplication::translate("Nav", "\345\257\274\350\210\252\350\207\263...", 0));
        pushButton_2->setText(QApplication::translate("Nav", "vicon\345\256\236\351\252\214\345\256\244", 0));
        pushButton_3->setText(QApplication::translate("Nav", "\350\214\266\345\256\244", 0));
        pushButton->setText(QApplication::translate("Nav", "\347\224\265\346\242\257\351\227\250\345\211\215", 0));
    } // retranslateUi

};

namespace Ui {
    class Nav: public Ui_Nav {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NAV_H
