#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>
#include <QMessageBox>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    position = 0;
    currentDirection = None; // 初始化currentDirection
    paused = false; // 初始化paused
    connect(ui->pushButton_4, &QPushButton::clicked, this, &MainWindow::on_pushButton_4_clicked);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    if (!paused) {
        currentDirection = Up;
        ui->pushButton->setText("上升中");
        updatePosition();
    }
}

void MainWindow::on_pushButton_3_clicked()
{
    if (!paused) {
        currentDirection = Down;
        ui->pushButton_2->setText("下降中");
        updatePosition();
    }
}

void MainWindow::on_pushButton_pause_clicked()
{
    paused = !paused; // 切换暂停状态
    ui->pushButton_3->setText(paused ? "暂停" : ""); // 更新方向显示
}

void MainWindow::updatePosition()
{
    if (currentDirection == Up) {
        position += 1;
    } else if (currentDirection == Down) {
        position -= 1;
    }


    if (!paused) {
        QTimer::singleShot(100, this, &MainWindow::updatePosition);
    }
}

void MainWindow::on_pushButton_4_clicked()
{
    // 在这里编写按钮点击后的处理代码

    // 示例：弹出一个消息框
    QMessageBox::information(this, "Button Clicked", "Button 4 was clicked!");

    // 示例：更改按钮文本
    ui->pushButton_4->setText("Clicked!");

    // 示例：执行某些其他操作
    // ...
}

void MainWindow::on_dial_valueChanged(int value)
{
    // 这里是处理值变化事件的逻辑代码
    // value 参数是控件当前的值，可以根据需要进行处理

    // 示例：将当前值显示在一个文本框中
    ui->label->setText(QString("Dial value: %1").arg(value));

    // 示例：根据值的大小改变窗口背景颜色
    if (value < 50) {
        this->setStyleSheet("background-color: lightblue;");
    } else {
        this->setStyleSheet("background-color: lightgreen;");
    }

}


void MainWindow::on_label_linkActivated(const QString &link)
{
    connect(ui->dial, &QDial::valueChanged, this, &MainWindow::on_dial_valueChanged);

}
