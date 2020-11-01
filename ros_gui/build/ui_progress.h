/********************************************************************************
** Form generated from reading UI file 'progress.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PROGRESS_H
#define UI_PROGRESS_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QProgressBar>

QT_BEGIN_NAMESPACE

class Ui_Progress
{
public:
    QProgressBar *progressBar;

    void setupUi(QDialog *Progress)
    {
        if (Progress->objectName().isEmpty())
            Progress->setObjectName(QStringLiteral("Progress"));
        Progress->resize(379, 57);
        progressBar = new QProgressBar(Progress);
        progressBar->setObjectName(QStringLiteral("progressBar"));
        progressBar->setGeometry(QRect(10, 20, 361, 23));
        progressBar->setValue(24);

        retranslateUi(Progress);

        QMetaObject::connectSlotsByName(Progress);
    } // setupUi

    void retranslateUi(QDialog *Progress)
    {
        Progress->setWindowTitle(QApplication::translate("Progress", "\350\277\233\345\272\246", 0));
    } // retranslateUi

};

namespace Ui {
    class Progress: public Ui_Progress {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PROGRESS_H
