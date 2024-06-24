/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QPushButton *upButton;
    QPushButton *stopButton;
    QPushButton *downButton;
    QPushButton *lightButton;
    QPushButton *uvButton;
    QPushButton *windButton;
    QPushButton *dryButton;
    QLabel *uvLabel;
    QLabel *windLabel;
    QLabel *dryLabel;
    QPushButton *windButtonTimer;
    QPushButton *dryButtonTimer;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(450, 245);
        MainWindow->setAutoFillBackground(false);
        MainWindow->setStyleSheet(QString::fromUtf8("QWidget {\n"
"    background-color: #ADD8E6; /* \346\265\205\350\223\235\350\211\262\350\203\214\346\231\257 */\n"
"}\n"
""));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        upButton = new QPushButton(centralwidget);
        upButton->setObjectName(QString::fromUtf8("upButton"));
        upButton->setGeometry(QRect(130, 0, 41, 31));
        upButton->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        stopButton = new QPushButton(centralwidget);
        stopButton->setObjectName(QString::fromUtf8("stopButton"));
        stopButton->setGeometry(QRect(210, 0, 61, 31));
        stopButton->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        downButton = new QPushButton(centralwidget);
        downButton->setObjectName(QString::fromUtf8("downButton"));
        downButton->setGeometry(QRect(320, 0, 41, 31));
        downButton->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        lightButton = new QPushButton(centralwidget);
        lightButton->setObjectName(QString::fromUtf8("lightButton"));
        lightButton->setGeometry(QRect(20, 50, 71, 31));
        lightButton->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        uvButton = new QPushButton(centralwidget);
        uvButton->setObjectName(QString::fromUtf8("uvButton"));
        uvButton->setGeometry(QRect(120, 50, 71, 31));
        QFont font;
        font.setFamily(QString::fromUtf8("Arial"));
        font.setPointSize(8);
        uvButton->setFont(font);
        uvButton->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        windButton = new QPushButton(centralwidget);
        windButton->setObjectName(QString::fromUtf8("windButton"));
        windButton->setGeometry(QRect(210, 50, 71, 31));
        windButton->setFont(font);
        windButton->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        dryButton = new QPushButton(centralwidget);
        dryButton->setObjectName(QString::fromUtf8("dryButton"));
        dryButton->setGeometry(QRect(300, 50, 81, 31));
        dryButton->setFont(font);
        dryButton->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        uvLabel = new QLabel(centralwidget);
        uvLabel->setObjectName(QString::fromUtf8("uvLabel"));
        uvLabel->setGeometry(QRect(120, 90, 81, 51));
        uvLabel->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        windLabel = new QLabel(centralwidget);
        windLabel->setObjectName(QString::fromUtf8("windLabel"));
        windLabel->setGeometry(QRect(210, 90, 81, 51));
        windLabel->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        dryLabel = new QLabel(centralwidget);
        dryLabel->setObjectName(QString::fromUtf8("dryLabel"));
        dryLabel->setGeometry(QRect(300, 90, 81, 51));
        dryLabel->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        windButtonTimer = new QPushButton(centralwidget);
        windButtonTimer->setObjectName(QString::fromUtf8("windButtonTimer"));
        windButtonTimer->setGeometry(QRect(120, 150, 111, 31));
        windButtonTimer->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        dryButtonTimer = new QPushButton(centralwidget);
        dryButtonTimer->setObjectName(QString::fromUtf8("dryButtonTimer"));
        dryButtonTimer->setGeometry(QRect(250, 150, 111, 31));
        dryButtonTimer->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"    background-color: #F4A460; /* \346\262\231\346\243\225\350\211\262\350\203\214\346\231\257 */\n"
"    color: white;\n"
"    border: 2px solid #000;\n"
"    border-radius: 5px;\n"
"    padding: 5px;\n"
"}\n"
"\n"
"QLabel {\n"
"    color: red; /* \347\272\242\350\211\262\345\255\227\344\275\223 */\n"
"    font-size: 20px;\n"
"    font-weight: bold;\n"
"}\n"
""));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 450, 21));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        upButton->setText(QCoreApplication::translate("MainWindow", "\344\270\212\345\215\207", nullptr));
        stopButton->setText(QCoreApplication::translate("MainWindow", "\346\232\202\345\201\234", nullptr));
        downButton->setText(QCoreApplication::translate("MainWindow", "\344\270\213\351\231\215", nullptr));
        lightButton->setText(QCoreApplication::translate("MainWindow", "\347\205\247\346\230\216\345\274\200/\345\205\263", nullptr));
        uvButton->setText(QCoreApplication::translate("MainWindow", "\347\264\253\345\244\226\347\272\277\345\274\200/\345\205\263", nullptr));
        windButton->setText(QCoreApplication::translate("MainWindow", "\351\243\216\345\271\262\345\274\200/\345\205\263", nullptr));
        dryButton->setText(QCoreApplication::translate("MainWindow", "\347\203\230\345\271\262\345\274\200/\345\205\263", nullptr));
        uvLabel->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        windLabel->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        dryLabel->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        windButtonTimer->setText(QCoreApplication::translate("MainWindow", "\351\243\216\345\271\262\345\256\232\346\227\2663s/\346\254\241", nullptr));
        dryButtonTimer->setText(QCoreApplication::translate("MainWindow", "\347\203\230\345\271\262\345\256\232\346\227\2663s/\346\254\241", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
