/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QHBoxLayout *horizontalLayout_5;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_10;
    QTabWidget *tabWidget;
    QWidget *tab;
    QVBoxLayout *verticalLayout_2;
    QPushButton *pushButton_4;
    QPushButton *pushButton_7;
    QFrame *line_2;
    QPushButton *pushButton_5;
    QPushButton *pushButton_8;
    QFrame *line;
    QPushButton *pushButton_6;
    QPushButton *pushButton_9;
    QPushButton *pushButton_10;
    QFrame *line_3;
    QPushButton *pushButton_12;
    QPushButton *pushButton_14;
    QWidget *widget;
    QVBoxLayout *verticalLayout_3;
    QPushButton *pushButton_21;
    QPushButton *pushButton_22;
    QFrame *line_4;
    QPushButton *pushButton_16;
    QPushButton *pushButton_17;
    QFrame *line_5;
    QPushButton *pushButton_15;
    QPushButton *pushButton_18;
    QPushButton *pushButton_23;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout;
    QSpacerItem *horizontalSpacer_2;
    QLabel *label;
    QProgressBar *progressBar;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *pushButton_24;
    QPushButton *pushButton_25;
    QPushButton *pushButton_29;
    QPushButton *pushButton_26;
    QPushButton *pushButton_27;
    QPushButton *pushButton_28;
    QPushButton *pushButton_30;
    QPushButton *pushButton_31;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout;
    QStatusBar *statusBar;
    QMenuBar *menuBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1223, 759);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        horizontalLayout_5 = new QHBoxLayout(centralWidget);
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        verticalLayout_2 = new QVBoxLayout(tab);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        pushButton_4 = new QPushButton(tab);
        pushButton_4->setObjectName(QStringLiteral("pushButton_4"));

        verticalLayout_2->addWidget(pushButton_4);

        pushButton_7 = new QPushButton(tab);
        pushButton_7->setObjectName(QStringLiteral("pushButton_7"));

        verticalLayout_2->addWidget(pushButton_7);

        line_2 = new QFrame(tab);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_2);

        pushButton_5 = new QPushButton(tab);
        pushButton_5->setObjectName(QStringLiteral("pushButton_5"));

        verticalLayout_2->addWidget(pushButton_5);

        pushButton_8 = new QPushButton(tab);
        pushButton_8->setObjectName(QStringLiteral("pushButton_8"));

        verticalLayout_2->addWidget(pushButton_8);

        line = new QFrame(tab);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line);

        pushButton_6 = new QPushButton(tab);
        pushButton_6->setObjectName(QStringLiteral("pushButton_6"));

        verticalLayout_2->addWidget(pushButton_6);

        pushButton_9 = new QPushButton(tab);
        pushButton_9->setObjectName(QStringLiteral("pushButton_9"));

        verticalLayout_2->addWidget(pushButton_9);

        pushButton_10 = new QPushButton(tab);
        pushButton_10->setObjectName(QStringLiteral("pushButton_10"));

        verticalLayout_2->addWidget(pushButton_10);

        line_3 = new QFrame(tab);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_3);

        pushButton_12 = new QPushButton(tab);
        pushButton_12->setObjectName(QStringLiteral("pushButton_12"));

        verticalLayout_2->addWidget(pushButton_12);

        pushButton_14 = new QPushButton(tab);
        pushButton_14->setObjectName(QStringLiteral("pushButton_14"));

        verticalLayout_2->addWidget(pushButton_14);

        tabWidget->addTab(tab, QString());
        widget = new QWidget();
        widget->setObjectName(QStringLiteral("widget"));
        verticalLayout_3 = new QVBoxLayout(widget);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        pushButton_21 = new QPushButton(widget);
        pushButton_21->setObjectName(QStringLiteral("pushButton_21"));

        verticalLayout_3->addWidget(pushButton_21);

        pushButton_22 = new QPushButton(widget);
        pushButton_22->setObjectName(QStringLiteral("pushButton_22"));

        verticalLayout_3->addWidget(pushButton_22);

        line_4 = new QFrame(widget);
        line_4->setObjectName(QStringLiteral("line_4"));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line_4);

        pushButton_16 = new QPushButton(widget);
        pushButton_16->setObjectName(QStringLiteral("pushButton_16"));

        verticalLayout_3->addWidget(pushButton_16);

        pushButton_17 = new QPushButton(widget);
        pushButton_17->setObjectName(QStringLiteral("pushButton_17"));

        verticalLayout_3->addWidget(pushButton_17);

        line_5 = new QFrame(widget);
        line_5->setObjectName(QStringLiteral("line_5"));
        line_5->setFrameShape(QFrame::HLine);
        line_5->setFrameShadow(QFrame::Sunken);

        verticalLayout_3->addWidget(line_5);

        pushButton_15 = new QPushButton(widget);
        pushButton_15->setObjectName(QStringLiteral("pushButton_15"));

        verticalLayout_3->addWidget(pushButton_15);

        pushButton_18 = new QPushButton(widget);
        pushButton_18->setObjectName(QStringLiteral("pushButton_18"));

        verticalLayout_3->addWidget(pushButton_18);

        tabWidget->addTab(widget, QString());

        verticalLayout_10->addWidget(tabWidget);

        pushButton_23 = new QPushButton(centralWidget);
        pushButton_23->setObjectName(QStringLiteral("pushButton_23"));

        verticalLayout_10->addWidget(pushButton_23);

        pushButton_2 = new QPushButton(centralWidget);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        verticalLayout_10->addWidget(pushButton_2);

        pushButton_3 = new QPushButton(centralWidget);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));

        verticalLayout_10->addWidget(pushButton_3);


        horizontalLayout_3->addLayout(verticalLayout_10);

        horizontalLayout_3->setStretch(0, 1);

        horizontalLayout_5->addLayout(horizontalLayout_3);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setPixmap(QPixmap(QString::fromUtf8(":/images/power.png")));

        horizontalLayout->addWidget(label);

        progressBar = new QProgressBar(centralWidget);
        progressBar->setObjectName(QStringLiteral("progressBar"));
        progressBar->setStyleSheet(QStringLiteral(""));
        progressBar->setValue(24);
        progressBar->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(progressBar);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setPixmap(QPixmap(QString::fromUtf8(":/images/power-v.png")));

        horizontalLayout->addWidget(label_2);

        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setPixmap(QPixmap(QString::fromUtf8(":/images/temperature.png")));

        horizontalLayout->addWidget(label_3);

        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout->addWidget(label_4);

        horizontalLayout->setStretch(0, 20);
        horizontalLayout->setStretch(1, 1);
        horizontalLayout->setStretch(2, 1);
        horizontalLayout->setStretch(3, 1);
        horizontalLayout->setStretch(4, 1);
        horizontalLayout->setStretch(5, 1);

        verticalLayout_6->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        pushButton_24 = new QPushButton(centralWidget);
        pushButton_24->setObjectName(QStringLiteral("pushButton_24"));

        horizontalLayout_2->addWidget(pushButton_24);

        pushButton_25 = new QPushButton(centralWidget);
        pushButton_25->setObjectName(QStringLiteral("pushButton_25"));

        horizontalLayout_2->addWidget(pushButton_25);

        pushButton_29 = new QPushButton(centralWidget);
        pushButton_29->setObjectName(QStringLiteral("pushButton_29"));

        horizontalLayout_2->addWidget(pushButton_29);

        pushButton_26 = new QPushButton(centralWidget);
        pushButton_26->setObjectName(QStringLiteral("pushButton_26"));

        horizontalLayout_2->addWidget(pushButton_26);

        pushButton_27 = new QPushButton(centralWidget);
        pushButton_27->setObjectName(QStringLiteral("pushButton_27"));

        horizontalLayout_2->addWidget(pushButton_27);

        pushButton_28 = new QPushButton(centralWidget);
        pushButton_28->setObjectName(QStringLiteral("pushButton_28"));

        horizontalLayout_2->addWidget(pushButton_28);

        pushButton_30 = new QPushButton(centralWidget);
        pushButton_30->setObjectName(QStringLiteral("pushButton_30"));

        horizontalLayout_2->addWidget(pushButton_30);

        pushButton_31 = new QPushButton(centralWidget);
        pushButton_31->setObjectName(QStringLiteral("pushButton_31"));

        horizontalLayout_2->addWidget(pushButton_31);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);


        verticalLayout_6->addLayout(horizontalLayout_2);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));

        verticalLayout_6->addLayout(verticalLayout);

        verticalLayout_6->setStretch(0, 1);
        verticalLayout_6->setStretch(1, 1);
        verticalLayout_6->setStretch(2, 30);

        horizontalLayout_5->addLayout(verticalLayout_6);

        horizontalLayout_5->setStretch(0, 1);
        horizontalLayout_5->setStretch(1, 10);
        MainWindow->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1223, 22));
        MainWindow->setMenuBar(menuBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        pushButton_4->setText(QApplication::translate("MainWindow", "\345\220\257\345\212\2502D\346\277\200\345\205\211", 0));
        pushButton_7->setText(QApplication::translate("MainWindow", "\345\205\263\351\227\255\346\277\200\345\205\211", 0));
        pushButton_5->setText(QApplication::translate("MainWindow", "\345\220\257\345\212\250\351\207\214\347\250\213\350\256\241", 0));
        pushButton_8->setText(QApplication::translate("MainWindow", "\345\205\263\351\227\255\351\207\214\347\250\213\350\256\241", 0));
        pushButton_6->setText(QApplication::translate("MainWindow", "\345\274\200\345\247\213\345\273\272\345\233\276", 0));
        pushButton_9->setText(QApplication::translate("MainWindow", "\344\277\235\345\255\230\345\234\260\345\233\276", 0));
        pushButton_10->setText(QApplication::translate("MainWindow", "\351\200\200\345\207\272\345\273\272\345\233\276", 0));
        pushButton_12->setText(QApplication::translate("MainWindow", "\345\220\257\345\212\250\345\257\274\350\210\252", 0));
        pushButton_14->setText(QApplication::translate("MainWindow", "\347\273\223\346\235\237\345\257\274\350\210\252", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "2D", 0));
        pushButton_21->setText(QApplication::translate("MainWindow", "\345\220\257\345\212\2503D\346\277\200\345\205\211", 0));
        pushButton_22->setText(QApplication::translate("MainWindow", "\345\205\263\351\227\255\346\277\200\345\205\211", 0));
        pushButton_16->setText(QApplication::translate("MainWindow", "\345\220\257\345\212\250\345\256\232\344\275\215", 0));
        pushButton_17->setText(QApplication::translate("MainWindow", "\347\273\223\346\235\237\345\256\232\344\275\215", 0));
        pushButton_15->setText(QApplication::translate("MainWindow", "\345\220\257\345\212\250\345\273\272\345\233\276", 0));
        pushButton_18->setText(QApplication::translate("MainWindow", "\347\273\223\346\235\237\345\273\272\345\233\276", 0));
        tabWidget->setTabText(tabWidget->indexOf(widget), QApplication::translate("MainWindow", "3D", 0));
        pushButton_23->setText(QApplication::translate("MainWindow", "\346\234\272\346\242\260\350\207\202\347\225\214\351\235\242", 0));
        pushButton_2->setText(QApplication::translate("MainWindow", "\345\220\257\345\212\250\351\201\245\346\216\247", 0));
        pushButton_3->setText(QApplication::translate("MainWindow", "\351\200\200\345\207\272\351\201\245\346\216\247", 0));
        label->setText(QString());
        label_2->setText(QString());
        label_3->setText(QString());
        label_4->setText(QString());
        pushButton_24->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\2562D\344\275\215\345\247\277", 0));
        pushButton_25->setText(QApplication::translate("MainWindow", "\350\256\276\347\275\2562D\345\257\274\350\210\252\347\202\271", 0));
        pushButton_29->setText(QApplication::translate("MainWindow", "\345\244\232\347\202\271\345\257\274\350\210\252", 0));
        pushButton_26->setText(QApplication::translate("MainWindow", "\346\267\273\345\212\240\347\246\201\350\241\214\345\214\272\345\237\237", 0));
        pushButton_27->setText(QApplication::translate("MainWindow", "\345\210\240\351\231\244\347\246\201\350\241\214\345\214\272\345\237\237", 0));
        pushButton_28->setText(QApplication::translate("MainWindow", "\350\257\255\344\271\211\346\240\207\350\256\260\345\214\272\345\237\237", 0));
        pushButton_30->setText(QApplication::translate("MainWindow", "test1", 0));
        pushButton_31->setText(QApplication::translate("MainWindow", "test2", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
