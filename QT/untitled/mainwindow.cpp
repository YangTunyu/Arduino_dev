#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    position = 0;
    currentDirection = None; // 初始化currentDirection
    paused = false; // 初始化paused
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
