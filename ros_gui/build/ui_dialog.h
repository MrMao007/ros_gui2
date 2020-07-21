/********************************************************************************
** Form generated from reading UI file 'dialog.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_H
#define UI_DIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Dialog
{
public:
    QHBoxLayout *horizontalLayout;
    QTabWidget *tabWidget;
    QWidget *tab;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLineEdit *lineEdit;
    QLabel *label_4;
    QLabel *label;
    QPushButton *pushButton_2;
    QPushButton *pushButton;
    QLabel *label_2;
    QLabel *label_6;
    QLabel *label_5;
    QLabel *label_3;
    QLineEdit *lineEdit_2;
    QLineEdit *lineEdit_3;
    QLineEdit *lineEdit_4;
    QPushButton *pushButton_4;
    QPushButton *pushButton_3;
    QWidget *tab_2;
    QWidget *gridLayoutWidget_4;
    QGridLayout *gridLayout_4;
    QLineEdit *lineEdit_13;
    QLabel *label_19;
    QLabel *label_20;
    QPushButton *pushButton_14;
    QPushButton *pushButton_13;
    QLabel *label_21;
    QLabel *label_22;
    QLabel *label_23;
    QLabel *label_24;
    QLineEdit *lineEdit_14;
    QLineEdit *lineEdit_15;
    QLineEdit *lineEdit_16;
    QPushButton *pushButton_15;
    QPushButton *pushButton_16;
    QWidget *tab_7;
    QPushButton *pushButton_17;
    QPushButton *pushButton_18;
    QWidget *gridLayoutWidget_5;
    QGridLayout *gridLayout_5;
    QPushButton *pushButton_20;
    QLabel *label_27;
    QLabel *label_30;
    QLineEdit *lineEdit_18;
    QLabel *label_26;
    QLineEdit *lineEdit_17;
    QLabel *label_7;
    QLabel *label_8;
    QLineEdit *lineEdit_5;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QStringLiteral("Dialog"));
        Dialog->resize(518, 289);
        horizontalLayout = new QHBoxLayout(Dialog);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        tabWidget = new QTabWidget(Dialog);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        gridLayoutWidget = new QWidget(tab);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(40, 50, 412, 88));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        lineEdit = new QLineEdit(gridLayoutWidget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setReadOnly(true);

        gridLayout->addWidget(lineEdit, 0, 2, 1, 1);

        label_4 = new QLabel(gridLayoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_4, 1, 0, 1, 1);

        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label, 0, 0, 1, 1);

        pushButton_2 = new QPushButton(gridLayoutWidget);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        gridLayout->addWidget(pushButton_2, 1, 5, 1, 1);

        pushButton = new QPushButton(gridLayoutWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        gridLayout->addWidget(pushButton, 0, 5, 1, 1);

        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_2, 0, 1, 1, 1);

        label_6 = new QLabel(gridLayoutWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_6, 1, 3, 1, 1);

        label_5 = new QLabel(gridLayoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_5, 1, 1, 1, 1);

        label_3 = new QLabel(gridLayoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(label_3, 0, 3, 1, 1);

        lineEdit_2 = new QLineEdit(gridLayoutWidget);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));
        lineEdit_2->setReadOnly(true);

        gridLayout->addWidget(lineEdit_2, 0, 4, 1, 1);

        lineEdit_3 = new QLineEdit(gridLayoutWidget);
        lineEdit_3->setObjectName(QStringLiteral("lineEdit_3"));
        lineEdit_3->setReadOnly(true);

        gridLayout->addWidget(lineEdit_3, 1, 2, 1, 1);

        lineEdit_4 = new QLineEdit(gridLayoutWidget);
        lineEdit_4->setObjectName(QStringLiteral("lineEdit_4"));
        lineEdit_4->setReadOnly(true);

        gridLayout->addWidget(lineEdit_4, 1, 4, 1, 1);

        gridLayout->setColumnStretch(0, 2);
        pushButton_4 = new QPushButton(tab);
        pushButton_4->setObjectName(QStringLiteral("pushButton_4"));
        pushButton_4->setGeometry(QRect(320, 170, 89, 25));
        pushButton_3 = new QPushButton(tab);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));
        pushButton_3->setGeometry(QRect(220, 170, 89, 25));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        gridLayoutWidget_4 = new QWidget(tab_2);
        gridLayoutWidget_4->setObjectName(QStringLiteral("gridLayoutWidget_4"));
        gridLayoutWidget_4->setGeometry(QRect(40, 50, 412, 88));
        gridLayout_4 = new QGridLayout(gridLayoutWidget_4);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        lineEdit_13 = new QLineEdit(gridLayoutWidget_4);
        lineEdit_13->setObjectName(QStringLiteral("lineEdit_13"));
        lineEdit_13->setReadOnly(true);

        gridLayout_4->addWidget(lineEdit_13, 0, 2, 1, 1);

        label_19 = new QLabel(gridLayoutWidget_4);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_19, 1, 0, 1, 1);

        label_20 = new QLabel(gridLayoutWidget_4);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_20, 0, 0, 1, 1);

        pushButton_14 = new QPushButton(gridLayoutWidget_4);
        pushButton_14->setObjectName(QStringLiteral("pushButton_14"));

        gridLayout_4->addWidget(pushButton_14, 1, 5, 1, 1);

        pushButton_13 = new QPushButton(gridLayoutWidget_4);
        pushButton_13->setObjectName(QStringLiteral("pushButton_13"));

        gridLayout_4->addWidget(pushButton_13, 0, 5, 1, 1);

        label_21 = new QLabel(gridLayoutWidget_4);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_21, 0, 1, 1, 1);

        label_22 = new QLabel(gridLayoutWidget_4);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_22, 1, 3, 1, 1);

        label_23 = new QLabel(gridLayoutWidget_4);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_23, 1, 1, 1, 1);

        label_24 = new QLabel(gridLayoutWidget_4);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(label_24, 0, 3, 1, 1);

        lineEdit_14 = new QLineEdit(gridLayoutWidget_4);
        lineEdit_14->setObjectName(QStringLiteral("lineEdit_14"));
        lineEdit_14->setReadOnly(true);

        gridLayout_4->addWidget(lineEdit_14, 0, 4, 1, 1);

        lineEdit_15 = new QLineEdit(gridLayoutWidget_4);
        lineEdit_15->setObjectName(QStringLiteral("lineEdit_15"));
        lineEdit_15->setReadOnly(true);

        gridLayout_4->addWidget(lineEdit_15, 1, 2, 1, 1);

        lineEdit_16 = new QLineEdit(gridLayoutWidget_4);
        lineEdit_16->setObjectName(QStringLiteral("lineEdit_16"));
        lineEdit_16->setReadOnly(true);

        gridLayout_4->addWidget(lineEdit_16, 1, 4, 1, 1);

        gridLayout_4->setColumnStretch(0, 2);
        pushButton_15 = new QPushButton(tab_2);
        pushButton_15->setObjectName(QStringLiteral("pushButton_15"));
        pushButton_15->setGeometry(QRect(220, 170, 89, 25));
        pushButton_16 = new QPushButton(tab_2);
        pushButton_16->setObjectName(QStringLiteral("pushButton_16"));
        pushButton_16->setGeometry(QRect(320, 170, 89, 25));
        tabWidget->addTab(tab_2, QString());
        tab_7 = new QWidget();
        tab_7->setObjectName(QStringLiteral("tab_7"));
        pushButton_17 = new QPushButton(tab_7);
        pushButton_17->setObjectName(QStringLiteral("pushButton_17"));
        pushButton_17->setGeometry(QRect(220, 170, 89, 25));
        pushButton_18 = new QPushButton(tab_7);
        pushButton_18->setObjectName(QStringLiteral("pushButton_18"));
        pushButton_18->setGeometry(QRect(320, 170, 89, 25));
        gridLayoutWidget_5 = new QWidget(tab_7);
        gridLayoutWidget_5->setObjectName(QStringLiteral("gridLayoutWidget_5"));
        gridLayoutWidget_5->setGeometry(QRect(40, 50, 412, 64));
        gridLayout_5 = new QGridLayout(gridLayoutWidget_5);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        pushButton_20 = new QPushButton(gridLayoutWidget_5);
        pushButton_20->setObjectName(QStringLiteral("pushButton_20"));

        gridLayout_5->addWidget(pushButton_20, 0, 5, 1, 1);

        label_27 = new QLabel(gridLayoutWidget_5);
        label_27->setObjectName(QStringLiteral("label_27"));
        label_27->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_27, 0, 1, 1, 1);

        label_30 = new QLabel(gridLayoutWidget_5);
        label_30->setObjectName(QStringLiteral("label_30"));
        label_30->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_30, 0, 3, 1, 1);

        lineEdit_18 = new QLineEdit(gridLayoutWidget_5);
        lineEdit_18->setObjectName(QStringLiteral("lineEdit_18"));
        lineEdit_18->setReadOnly(true);

        gridLayout_5->addWidget(lineEdit_18, 0, 4, 1, 1);

        label_26 = new QLabel(gridLayoutWidget_5);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setAlignment(Qt::AlignCenter);

        gridLayout_5->addWidget(label_26, 0, 0, 1, 1);

        lineEdit_17 = new QLineEdit(gridLayoutWidget_5);
        lineEdit_17->setObjectName(QStringLiteral("lineEdit_17"));
        lineEdit_17->setReadOnly(true);

        gridLayout_5->addWidget(lineEdit_17, 0, 2, 1, 1);

        label_7 = new QLabel(gridLayoutWidget_5);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout_5->addWidget(label_7, 1, 0, 1, 1);

        label_8 = new QLabel(gridLayoutWidget_5);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout_5->addWidget(label_8, 1, 1, 1, 1);

        lineEdit_5 = new QLineEdit(gridLayoutWidget_5);
        lineEdit_5->setObjectName(QStringLiteral("lineEdit_5"));

        gridLayout_5->addWidget(lineEdit_5, 1, 2, 1, 1);

        gridLayout_5->setColumnStretch(0, 2);
        tabWidget->addTab(tab_7, QString());

        horizontalLayout->addWidget(tabWidget);


        retranslateUi(Dialog);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QApplication::translate("Dialog", "\346\267\273\345\212\240\347\246\201\350\241\214\345\214\272\345\237\237", 0));
        label_4->setText(QApplication::translate("Dialog", "\347\273\210\347\202\271", 0));
        label->setText(QApplication::translate("Dialog", "\350\265\267\347\202\271", 0));
        pushButton_2->setText(QApplication::translate("Dialog", "\351\200\211\345\217\226\347\273\210\347\202\271", 0));
        pushButton->setText(QApplication::translate("Dialog", "\351\200\211\345\217\226\350\265\267\347\202\271", 0));
        label_2->setText(QApplication::translate("Dialog", "X:", 0));
        label_6->setText(QApplication::translate("Dialog", "Y:", 0));
        label_5->setText(QApplication::translate("Dialog", "X:", 0));
        label_3->setText(QApplication::translate("Dialog", "Y:", 0));
        pushButton_4->setText(QApplication::translate("Dialog", "\345\217\226\346\266\210", 0));
        pushButton_3->setText(QApplication::translate("Dialog", "\347\241\256\345\256\232", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("Dialog", "\347\237\251\345\275\242", 0));
        label_19->setText(QApplication::translate("Dialog", "\347\273\210\347\202\271", 0));
        label_20->setText(QApplication::translate("Dialog", "\350\265\267\347\202\271", 0));
        pushButton_14->setText(QApplication::translate("Dialog", "\351\200\211\345\217\226\347\273\210\347\202\271", 0));
        pushButton_13->setText(QApplication::translate("Dialog", "\351\200\211\345\217\226\350\265\267\347\202\271", 0));
        label_21->setText(QApplication::translate("Dialog", "X:", 0));
        label_22->setText(QApplication::translate("Dialog", "Y:", 0));
        label_23->setText(QApplication::translate("Dialog", "X:", 0));
        label_24->setText(QApplication::translate("Dialog", "Y:", 0));
        pushButton_15->setText(QApplication::translate("Dialog", "\347\241\256\345\256\232", 0));
        pushButton_16->setText(QApplication::translate("Dialog", "\345\217\226\346\266\210", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("Dialog", "\347\233\264\347\272\277", 0));
        pushButton_17->setText(QApplication::translate("Dialog", "\347\241\256\345\256\232", 0));
        pushButton_18->setText(QApplication::translate("Dialog", "\345\217\226\346\266\210", 0));
        pushButton_20->setText(QApplication::translate("Dialog", "\351\200\211\345\217\226\347\202\271", 0));
        label_27->setText(QApplication::translate("Dialog", "X:", 0));
        label_30->setText(QApplication::translate("Dialog", "Y:", 0));
        label_26->setText(QApplication::translate("Dialog", "\345\234\206\345\277\203", 0));
        label_7->setText(QApplication::translate("Dialog", "\345\215\212\345\276\204", 0));
        label_8->setText(QApplication::translate("Dialog", "\357\274\232", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_7), QApplication::translate("Dialog", "\345\234\206", 0));
    } // retranslateUi

};

namespace Ui {
    class Dialog: public Ui_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_H
