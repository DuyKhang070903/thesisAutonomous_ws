/********************************************************************************
** Form generated from reading UI file 'robot_gui.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROBOT_GUI_H
#define UI_ROBOT_GUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_robot_gui
{
public:

    void setupUi(QWidget *robot_gui)
    {
        if (robot_gui->objectName().isEmpty())
            robot_gui->setObjectName(QString::fromUtf8("robot_gui"));
        robot_gui->resize(400, 300);

        retranslateUi(robot_gui);

        QMetaObject::connectSlotsByName(robot_gui);
    } // setupUi

    void retranslateUi(QWidget *robot_gui)
    {
        robot_gui->setWindowTitle(QApplication::translate("robot_gui", "Form", nullptr));
    } // retranslateUi

};

namespace Ui {
    class robot_gui: public Ui_robot_gui {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROBOT_GUI_H
