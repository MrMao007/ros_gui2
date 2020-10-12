/********************************************************************************
** Form generated from reading UI file 'semantic.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SEMANTIC_H
#define UI_SEMANTIC_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Semantic
{
public:
    QPushButton *pushButton;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLineEdit *lineEdit;
    QLabel *label;
    QPushButton *pushButton_2;

    void setupUi(QDialog *Semantic)
    {
        if (Semantic->objectName().isEmpty())
            Semantic->setObjectName(QStringLiteral("Semantic"));
        Semantic->resize(518, 289);
        pushButton = new QPushButton(Semantic);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(260, 180, 89, 25));
        gridLayoutWidget = new QWidget(Semantic);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(130, 90, 251, 64));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        lineEdit = new QLineEdit(gridLayoutWidget);
        lineEdit->setObjectName(QStringLiteral("lineEdit"));

        gridLayout->addWidget(lineEdit, 1, 1, 1, 1);

        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 1, 0, 1, 1);

        pushButton_2 = new QPushButton(gridLayoutWidget);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));

        gridLayout->addWidget(pushButton_2, 0, 1, 1, 1);


        retranslateUi(Semantic);

        QMetaObject::connectSlotsByName(Semantic);
    } // setupUi

    void retranslateUi(QDialog *Semantic)
    {
        Semantic->setWindowTitle(QApplication::translate("Semantic", "\350\257\255\344\271\211\346\240\207\350\256\260\345\214\272\345\237\237", 0));
        pushButton->setText(QApplication::translate("Semantic", "\347\241\256\345\256\232", 0));
        label->setText(QApplication::translate("Semantic", "\346\240\207\350\256\260\345\206\205\345\256\271\357\274\232", 0));
        pushButton_2->setText(QApplication::translate("Semantic", "\351\200\211\345\217\226\346\240\207\350\256\260\344\275\215\347\275\256", 0));
    } // retranslateUi

};

namespace Ui {
    class Semantic: public Ui_Semantic {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SEMANTIC_H
