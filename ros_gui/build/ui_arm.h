/********************************************************************************
** Form generated from reading UI file 'arm.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ARM_H
#define UI_ARM_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_arm
{
public:
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout;
    QWidget *widget;
    QVBoxLayout *verticalLayout_7;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_4;
    QLabel *label_18;
    QFormLayout *formLayout;
    QLabel *label_19;
    QLineEdit *lineEdit;
    QLabel *label_22;
    QLineEdit *lineEdit_2;
    QLabel *label_20;
    QLineEdit *lineEdit_3;
    QLabel *label_21;
    QLineEdit *lineEdit_4;
    QLabel *label_23;
    QLineEdit *lineEdit_5;
    QLabel *label_24;
    QLineEdit *lineEdit_6;
    QLabel *label_25;
    QLineEdit *lineEdit_7;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout_5;
    QLabel *label_10;
    QFormLayout *formLayout_2;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *label_16;
    QLineEdit *lineEdit_9;
    QLineEdit *lineEdit_10;
    QLineEdit *lineEdit_11;
    QLineEdit *lineEdit_12;
    QLineEdit *lineEdit_13;
    QLineEdit *lineEdit_14;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_26;
    QLineEdit *lineEdit_8;
    QSpacerItem *horizontalSpacer_2;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QLabel *label_17;
    QPushButton *pushButton;
    QVBoxLayout *verticalLayout_3;

    void setupUi(QWidget *arm)
    {
        if (arm->objectName().isEmpty())
            arm->setObjectName(QStringLiteral("arm"));
        arm->resize(408, 651);
        gridLayout = new QGridLayout(arm);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        widget = new QWidget(arm);
        widget->setObjectName(QStringLiteral("widget"));
        verticalLayout_7 = new QVBoxLayout(widget);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        label_18 = new QLabel(widget);
        label_18->setObjectName(QStringLiteral("label_18"));
        QFont font;
        font.setPointSize(13);
        label_18->setFont(font);

        verticalLayout_4->addWidget(label_18);

        formLayout = new QFormLayout();
        formLayout->setObjectName(QStringLiteral("formLayout"));
        label_19 = new QLabel(widget);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setFont(font);

        formLayout->setWidget(0, QFormLayout::LabelRole, label_19);

        lineEdit = new QLineEdit(widget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setReadOnly(true);

        formLayout->setWidget(0, QFormLayout::FieldRole, lineEdit);

        label_22 = new QLabel(widget);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setFont(font);

        formLayout->setWidget(1, QFormLayout::LabelRole, label_22);

        lineEdit_2 = new QLineEdit(widget);
        lineEdit_2->setObjectName(QStringLiteral("lineEdit_2"));
        lineEdit_2->setReadOnly(true);

        formLayout->setWidget(1, QFormLayout::FieldRole, lineEdit_2);

        label_20 = new QLabel(widget);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setFont(font);

        formLayout->setWidget(2, QFormLayout::LabelRole, label_20);

        lineEdit_3 = new QLineEdit(widget);
        lineEdit_3->setObjectName(QStringLiteral("lineEdit_3"));
        lineEdit_3->setReadOnly(true);

        formLayout->setWidget(2, QFormLayout::FieldRole, lineEdit_3);

        label_21 = new QLabel(widget);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setFont(font);

        formLayout->setWidget(3, QFormLayout::LabelRole, label_21);

        lineEdit_4 = new QLineEdit(widget);
        lineEdit_4->setObjectName(QStringLiteral("lineEdit_4"));
        lineEdit_4->setReadOnly(true);

        formLayout->setWidget(3, QFormLayout::FieldRole, lineEdit_4);

        label_23 = new QLabel(widget);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setFont(font);

        formLayout->setWidget(4, QFormLayout::LabelRole, label_23);

        lineEdit_5 = new QLineEdit(widget);
        lineEdit_5->setObjectName(QStringLiteral("lineEdit_5"));
        lineEdit_5->setReadOnly(true);

        formLayout->setWidget(4, QFormLayout::FieldRole, lineEdit_5);

        label_24 = new QLabel(widget);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setFont(font);

        formLayout->setWidget(5, QFormLayout::LabelRole, label_24);

        lineEdit_6 = new QLineEdit(widget);
        lineEdit_6->setObjectName(QStringLiteral("lineEdit_6"));
        lineEdit_6->setReadOnly(true);

        formLayout->setWidget(5, QFormLayout::FieldRole, lineEdit_6);

        label_25 = new QLabel(widget);
        label_25->setObjectName(QStringLiteral("label_25"));
        label_25->setFont(font);

        formLayout->setWidget(6, QFormLayout::LabelRole, label_25);

        lineEdit_7 = new QLineEdit(widget);
        lineEdit_7->setObjectName(QStringLiteral("lineEdit_7"));
        lineEdit_7->setReadOnly(true);

        formLayout->setWidget(6, QFormLayout::FieldRole, lineEdit_7);


        verticalLayout_4->addLayout(formLayout);

        verticalLayout_4->setStretch(0, 1);
        verticalLayout_4->setStretch(1, 11);

        horizontalLayout_3->addLayout(verticalLayout_4);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        label_10 = new QLabel(widget);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setFont(font);

        verticalLayout_5->addWidget(label_10);

        formLayout_2 = new QFormLayout();
        formLayout_2->setObjectName(QStringLiteral("formLayout_2"));
        formLayout_2->setLabelAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_11 = new QLabel(widget);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setFont(font);

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_11);

        label_12 = new QLabel(widget);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setFont(font);

        formLayout_2->setWidget(1, QFormLayout::LabelRole, label_12);

        label_13 = new QLabel(widget);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setFont(font);

        formLayout_2->setWidget(2, QFormLayout::LabelRole, label_13);

        label_14 = new QLabel(widget);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setFont(font);

        formLayout_2->setWidget(3, QFormLayout::LabelRole, label_14);

        label_15 = new QLabel(widget);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setFont(font);

        formLayout_2->setWidget(4, QFormLayout::LabelRole, label_15);

        label_16 = new QLabel(widget);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setFont(font);

        formLayout_2->setWidget(5, QFormLayout::LabelRole, label_16);

        lineEdit_9 = new QLineEdit(widget);
        lineEdit_9->setObjectName(QStringLiteral("lineEdit_9"));
        lineEdit_9->setReadOnly(true);

        formLayout_2->setWidget(0, QFormLayout::FieldRole, lineEdit_9);

        lineEdit_10 = new QLineEdit(widget);
        lineEdit_10->setObjectName(QStringLiteral("lineEdit_10"));
        lineEdit_10->setReadOnly(true);

        formLayout_2->setWidget(1, QFormLayout::FieldRole, lineEdit_10);

        lineEdit_11 = new QLineEdit(widget);
        lineEdit_11->setObjectName(QStringLiteral("lineEdit_11"));
        lineEdit_11->setReadOnly(true);

        formLayout_2->setWidget(2, QFormLayout::FieldRole, lineEdit_11);

        lineEdit_12 = new QLineEdit(widget);
        lineEdit_12->setObjectName(QStringLiteral("lineEdit_12"));
        lineEdit_12->setReadOnly(true);

        formLayout_2->setWidget(3, QFormLayout::FieldRole, lineEdit_12);

        lineEdit_13 = new QLineEdit(widget);
        lineEdit_13->setObjectName(QStringLiteral("lineEdit_13"));
        lineEdit_13->setReadOnly(true);

        formLayout_2->setWidget(4, QFormLayout::FieldRole, lineEdit_13);

        lineEdit_14 = new QLineEdit(widget);
        lineEdit_14->setObjectName(QStringLiteral("lineEdit_14"));
        lineEdit_14->setReadOnly(true);

        formLayout_2->setWidget(5, QFormLayout::FieldRole, lineEdit_14);


        verticalLayout_5->addLayout(formLayout_2);

        verticalLayout_5->setStretch(0, 1);
        verticalLayout_5->setStretch(1, 11);

        horizontalLayout_3->addLayout(verticalLayout_5);

        horizontalLayout_3->setStretch(0, 1);
        horizontalLayout_3->setStretch(1, 1);
        horizontalLayout_3->setStretch(2, 1);

        verticalLayout_6->addLayout(horizontalLayout_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_26 = new QLabel(widget);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setFont(font);

        horizontalLayout_2->addWidget(label_26);

        lineEdit_8 = new QLineEdit(widget);
        lineEdit_8->setObjectName(QStringLiteral("lineEdit_8"));

        horizontalLayout_2->addWidget(lineEdit_8);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        horizontalLayout_2->setStretch(0, 1);
        horizontalLayout_2->setStretch(1, 1);
        horizontalLayout_2->setStretch(2, 2);

        verticalLayout_6->addLayout(horizontalLayout_2);


        verticalLayout_7->addLayout(verticalLayout_6);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        pushButton_2 = new QPushButton(widget);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        horizontalLayout->addWidget(pushButton_2);

        pushButton_3 = new QPushButton(widget);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));

        horizontalLayout->addWidget(pushButton_3);

        label_17 = new QLabel(widget);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setStyleSheet(QLatin1String("color: rgb(204, 0, 0);\n"
"border:2px solid rgb(0, 0, 0);\n"
""));

        horizontalLayout->addWidget(label_17);

        pushButton = new QPushButton(widget);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        horizontalLayout->addWidget(pushButton);


        verticalLayout_7->addLayout(horizontalLayout);


        verticalLayout->addWidget(widget);


        gridLayout->addLayout(verticalLayout, 1, 0, 1, 1);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));

        gridLayout->addLayout(verticalLayout_3, 0, 0, 1, 1);

        gridLayout->setRowStretch(0, 1);
        gridLayout->setColumnStretch(0, 1);

        retranslateUi(arm);

        QMetaObject::connectSlotsByName(arm);
    } // setupUi

    void retranslateUi(QWidget *arm)
    {
        arm->setWindowTitle(QApplication::translate("arm", "Robotic arm", 0));
        label_18->setText(QApplication::translate("arm", "Joint", 0));
        label_19->setText(QApplication::translate("arm", "J1", 0));
        lineEdit->setText(QApplication::translate("arm", " \302\260", 0));
        label_22->setText(QApplication::translate("arm", "J2", 0));
        lineEdit_2->setText(QApplication::translate("arm", "\302\260", 0));
        label_20->setText(QApplication::translate("arm", "J3", 0));
        lineEdit_3->setText(QApplication::translate("arm", "\302\260", 0));
        label_21->setText(QApplication::translate("arm", "J4", 0));
        lineEdit_4->setText(QApplication::translate("arm", "\302\260", 0));
        label_23->setText(QApplication::translate("arm", "J5", 0));
        lineEdit_5->setText(QApplication::translate("arm", "\302\260", 0));
        label_24->setText(QApplication::translate("arm", "J6", 0));
        lineEdit_6->setText(QApplication::translate("arm", "\302\260", 0));
        label_25->setText(QApplication::translate("arm", "J7", 0));
        lineEdit_7->setText(QApplication::translate("arm", "\302\260", 0));
        label_10->setText(QApplication::translate("arm", "End_effector", 0));
        label_11->setText(QApplication::translate("arm", "X", 0));
        label_12->setText(QApplication::translate("arm", "Y", 0));
        label_13->setText(QApplication::translate("arm", "Z", 0));
        label_14->setText(QApplication::translate("arm", "R", 0));
        label_15->setText(QApplication::translate("arm", "P", 0));
        label_16->setText(QApplication::translate("arm", "YAW", 0));
        lineEdit_9->setText(QApplication::translate("arm", "0.12", 0));
        lineEdit_10->setText(QApplication::translate("arm", "0.10", 0));
        lineEdit_11->setText(QApplication::translate("arm", "0.2", 0));
        lineEdit_12->setText(QApplication::translate("arm", "0", 0));
        lineEdit_13->setText(QApplication::translate("arm", "0", 0));
        lineEdit_14->setText(QApplication::translate("arm", "0", 0));
        label_26->setText(QApplication::translate("arm", "Gripper distance", 0));
        pushButton_2->setText(QApplication::translate("arm", "\350\277\236\346\216\245", 0));
        pushButton_3->setText(QApplication::translate("arm", "\346\226\255\345\274\200\350\277\236\346\216\245", 0));
        label_17->setText(QString());
        pushButton->setText(QApplication::translate("arm", "\350\277\224\345\233\236\344\270\273\347\225\214\351\235\242", 0));
    } // retranslateUi

};

namespace Ui {
    class arm: public Ui_arm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ARM_H
