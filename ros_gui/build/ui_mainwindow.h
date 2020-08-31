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
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
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
    QVBoxLayout *_2;
    QGridLayout *gridLayout;
    QLabel *label_7;
    QRadioButton *radioButton_2;
    QRadioButton *radioButton;
    QLabel *label_5;
    QRadioButton *radioButton_3;
    QLabel *label_6;
    QRadioButton *radioButton_4;
    QLabel *label_8;
    QSpacerItem *verticalSpacer;
    QWidget *widget;
    QVBoxLayout *verticalLayout_3;
    QGridLayout *gridLayout_3;
    QRadioButton *radioButton_6;
    QRadioButton *radioButton_7;
    QLabel *label_10;
    QLabel *label_11;
    QRadioButton *radioButton_8;
    QLabel *label_13;
    QLabel *label_12;
    QRadioButton *radioButton_9;
    QRadioButton *radioButton_10;
    QLabel *label_14;
    QLineEdit *lineEdit;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QSpacerItem *verticalSpacer_2;
    QGridLayout *gridLayout_2;
    QRadioButton *radioButton_5;
    QLabel *label_9;
    QPushButton *pushButton_23;
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
        MainWindow->resize(1223, 635);
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
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy);
        tabWidget->setTabPosition(QTabWidget::North);
        tabWidget->setTabShape(QTabWidget::Rounded);
        tabWidget->setIconSize(QSize(16, 16));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        tab->setStyleSheet(QStringLiteral("background-color: rgb(238, 238, 236);"));
        _2 = new QVBoxLayout(tab);
        _2->setSpacing(0);
        _2->setContentsMargins(11, 11, 11, 11);
        _2->setObjectName(QStringLiteral("_2"));
        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label_7 = new QLabel(tab);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(label_7, 2, 1, 1, 1);

        radioButton_2 = new QRadioButton(tab);
        radioButton_2->setObjectName(QStringLiteral("radioButton_2"));
        radioButton_2->setMinimumSize(QSize(25, 25));
        radioButton_2->setAutoExclusive(false);

        gridLayout->addWidget(radioButton_2, 1, 0, 1, 1);

        radioButton = new QRadioButton(tab);
        radioButton->setObjectName(QStringLiteral("radioButton"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(radioButton->sizePolicy().hasHeightForWidth());
        radioButton->setSizePolicy(sizePolicy1);
        radioButton->setMinimumSize(QSize(25, 25));
        radioButton->setStyleSheet(QStringLiteral(""));
        radioButton->setChecked(false);
        radioButton->setAutoExclusive(false);

        gridLayout->addWidget(radioButton, 0, 0, 1, 1);

        label_5 = new QLabel(tab);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setTextFormat(Qt::PlainText);
        label_5->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(label_5, 0, 1, 1, 1);

        radioButton_3 = new QRadioButton(tab);
        radioButton_3->setObjectName(QStringLiteral("radioButton_3"));
        radioButton_3->setMinimumSize(QSize(25, 25));
        radioButton_3->setCheckable(true);
        radioButton_3->setAutoExclusive(false);

        gridLayout->addWidget(radioButton_3, 2, 0, 1, 1);

        label_6 = new QLabel(tab);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(label_6, 1, 1, 1, 1);

        radioButton_4 = new QRadioButton(tab);
        radioButton_4->setObjectName(QStringLiteral("radioButton_4"));
        radioButton_4->setMinimumSize(QSize(25, 25));
        radioButton_4->setCheckable(true);
        radioButton_4->setAutoExclusive(false);

        gridLayout->addWidget(radioButton_4, 3, 0, 1, 1);

        label_8 = new QLabel(tab);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout->addWidget(label_8, 3, 1, 1, 1);


        _2->addLayout(gridLayout);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        _2->addItem(verticalSpacer);

        tabWidget->addTab(tab, QString());
        widget = new QWidget();
        widget->setObjectName(QStringLiteral("widget"));
        widget->setStyleSheet(QStringLiteral("background-color: rgb(238, 238, 236);"));
        verticalLayout_3 = new QVBoxLayout(widget);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        gridLayout_3 = new QGridLayout();
        gridLayout_3->setSpacing(6);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        radioButton_6 = new QRadioButton(widget);
        radioButton_6->setObjectName(QStringLiteral("radioButton_6"));
        radioButton_6->setMinimumSize(QSize(25, 25));
        radioButton_6->setAutoExclusive(false);

        gridLayout_3->addWidget(radioButton_6, 0, 0, 1, 1);

        radioButton_7 = new QRadioButton(widget);
        radioButton_7->setObjectName(QStringLiteral("radioButton_7"));
        radioButton_7->setMinimumSize(QSize(25, 25));
        radioButton_7->setCheckable(true);
        radioButton_7->setAutoExclusive(false);

        gridLayout_3->addWidget(radioButton_7, 1, 0, 1, 1);

        label_10 = new QLabel(widget);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_3->addWidget(label_10, 0, 1, 1, 1);

        label_11 = new QLabel(widget);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_3->addWidget(label_11, 1, 1, 1, 1);

        radioButton_8 = new QRadioButton(widget);
        radioButton_8->setObjectName(QStringLiteral("radioButton_8"));
        radioButton_8->setMinimumSize(QSize(25, 25));
        radioButton_8->setCheckable(true);
        radioButton_8->setAutoExclusive(false);

        gridLayout_3->addWidget(radioButton_8, 2, 0, 1, 1);

        label_13 = new QLabel(widget);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_3->addWidget(label_13, 3, 1, 1, 1);

        label_12 = new QLabel(widget);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_3->addWidget(label_12, 2, 1, 1, 1);

        radioButton_9 = new QRadioButton(widget);
        radioButton_9->setObjectName(QStringLiteral("radioButton_9"));
        radioButton_9->setMinimumSize(QSize(25, 25));
        radioButton_9->setAutoExclusive(false);

        gridLayout_3->addWidget(radioButton_9, 3, 0, 1, 1);

        radioButton_10 = new QRadioButton(widget);
        radioButton_10->setObjectName(QStringLiteral("radioButton_10"));
        radioButton_10->setMinimumSize(QSize(25, 25));
        radioButton_10->setAutoExclusive(false);

        gridLayout_3->addWidget(radioButton_10, 4, 0, 1, 1);

        label_14 = new QLabel(widget);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_3->addWidget(label_14, 4, 1, 1, 1);


        verticalLayout_3->addLayout(gridLayout_3);

        lineEdit = new QLineEdit(widget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));
        lineEdit->setStyleSheet(QStringLiteral("background-color: rgb(255, 255, 255);"));
        lineEdit->setReadOnly(true);

        verticalLayout_3->addWidget(lineEdit);

        pushButton = new QPushButton(widget);
        pushButton->setObjectName(QStringLiteral("pushButton"));

        verticalLayout_3->addWidget(pushButton);

        pushButton_2 = new QPushButton(widget);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        verticalLayout_3->addWidget(pushButton_2);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_2);

        tabWidget->addTab(widget, QString());

        verticalLayout_10->addWidget(tabWidget);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setSpacing(6);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        radioButton_5 = new QRadioButton(centralWidget);
        radioButton_5->setObjectName(QStringLiteral("radioButton_5"));
        radioButton_5->setMinimumSize(QSize(25, 25));
        radioButton_5->setAutoExclusive(false);

        gridLayout_2->addWidget(radioButton_5, 0, 0, 1, 1);

        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        gridLayout_2->addWidget(label_9, 0, 1, 1, 1);


        verticalLayout_10->addLayout(gridLayout_2);

        pushButton_23 = new QPushButton(centralWidget);
        pushButton_23->setObjectName(QStringLiteral("pushButton_23"));

        verticalLayout_10->addWidget(pushButton_23);


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

        tabWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        label_7->setText(QApplication::translate("MainWindow", "\345\273\272\345\233\276", 0));
        radioButton_2->setText(QString());
        radioButton->setText(QString());
        label_5->setText(QApplication::translate("MainWindow", "2D\346\277\200\345\205\211", 0));
        radioButton_3->setText(QString());
        label_6->setText(QApplication::translate("MainWindow", "\351\207\214\347\250\213\350\256\241", 0));
        radioButton_4->setText(QString());
        label_8->setText(QApplication::translate("MainWindow", "\345\257\274\350\210\252", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("MainWindow", "2D", 0));
        radioButton_6->setText(QString());
        radioButton_7->setText(QString());
        label_10->setText(QApplication::translate("MainWindow", "3D\346\277\200\345\205\211", 0));
        label_11->setText(QApplication::translate("MainWindow", "\345\256\232\344\275\215", 0));
        radioButton_8->setText(QString());
        label_13->setText(QApplication::translate("MainWindow", "\350\256\260\345\275\225\350\267\257\345\276\204", 0));
        label_12->setText(QApplication::translate("MainWindow", "\345\273\272\345\233\276", 0));
        radioButton_9->setText(QString());
        radioButton_10->setText(QString());
        label_14->setText(QApplication::translate("MainWindow", "\350\267\237\350\270\252\350\267\257\345\276\204", 0));
        lineEdit->setText(QString());
        pushButton->setText(QApplication::translate("MainWindow", "\351\200\211\346\213\251\345\234\260\345\233\276", 0));
        pushButton_2->setText(QApplication::translate("MainWindow", "\346\211\223\345\274\200\345\234\260\345\233\276", 0));
        tabWidget->setTabText(tabWidget->indexOf(widget), QApplication::translate("MainWindow", "3D", 0));
        radioButton_5->setText(QString());
        label_9->setText(QApplication::translate("MainWindow", "\351\201\245\346\216\247", 0));
        pushButton_23->setText(QApplication::translate("MainWindow", "\346\234\272\346\242\260\350\207\202\347\225\214\351\235\242", 0));
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
