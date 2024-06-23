#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , uvTimeRemaining(15) // 初始时间为15秒
    , windTimeRemaining(15) // 初始时间为15秒
    , dryTimeRemaining(15) // 初始时间为15秒
    , lightState(false) // 初始化照明状态为关闭
    , uvState(false) // 初始化紫外线状态为关闭
    , windState(false) // 初始化风干状态为关闭
    , dryState(false) // 初始化烘干状态为关闭
{
    ui->setupUi(this);

    connect(&uvTimer, &QTimer::timeout, this, &MainWindow::updateUVTimer);
    connect(&windTimer, &QTimer::timeout, this, &MainWindow::updateWindTimer);
    connect(&dryTimer, &QTimer::timeout, this, &MainWindow::updateDryTimer);

    // 初始化按钮文本
    ui->lightButton->setText("照明 开");
    ui->uvButton->setText("紫外线 开");
    ui->windButton->setText("风干 开");
    ui->dryButton->setText("烘干 开");

    // 隐藏倒计时显示
    ui->uvLabel->hide();
    ui->windLabel->hide();
    ui->dryLabel->hide();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_lightButton_clicked()
{
    lightState = !lightState; // 切换状态
    if (lightState) {
        ui->lightButton->setText("照明 关");
    } else {
        ui->lightButton->setText("照明 开");
    }
}

void MainWindow::on_uvButton_clicked()
{
    uvState = !uvState; // 切换状态
    if (uvState) {
        //写上通过wifi发送紫外开的指令
        uvTimeRemaining = 15; // 重置倒计时
        ui->uvLabel->setText(QString::number(uvTimeRemaining));
        ui->uvLabel->show();
        uvTimer.start(1000); // 每秒更新一次
        ui->uvButton->setText("紫外线 关");
    } else {
        //写上通过wifi发送紫外关的指令
        uvTimer.stop();
        ui->uvLabel->hide();
        ui->uvButton->setText("紫外线 开");
    }
}

void MainWindow::on_windButton_clicked()
{
    windState = !windState; // 切换风干的状态
    if (windState) {
        //写上通过wifi发送风干开的指令
        windTimeRemaining = 15; // 重置倒计时
        ui->windLabel->setText(QString::number(windTimeRemaining));
        ui->windLabel->show();
        windTimer.start(1000); // 每秒更新一次
        ui->windButton->setText("风干 关");

    } else {
        //写上通过WiFi发送风干关的指令
        windTimer.stop();
        ui->windLabel->hide();
        ui->windButton->setText("风干 开");

    }
}

void MainWindow::on_dryButton_clicked()
{
    dryState = !dryState; // 切换状态
    if (dryState) {
        //写上通过WiFi发送烘干开的指令
        dryTimeRemaining = 15; // 重置倒计时
        ui->dryLabel->setText(QString::number(dryTimeRemaining));
        ui->dryLabel->show();
        dryTimer.start(1000); // 每秒更新一次
        ui->dryButton->setText("烘干 关");
    } else {
        //写上通过WiFi发送烘干关的指令
        dryTimer.stop();
        ui->dryLabel->hide();
        ui->dryButton->setText("烘干 开");
    }
}

void MainWindow::on_stopButton_clicked()
{
    uvTimer.stop();
    windTimer.stop();
    dryTimer.stop();

    // 重置所有开关状态和按钮文本
    uvState = false;
    windState = false;
    dryState = false;
    ui->uvButton->setText("紫外线 开");
    ui->windButton->setText("风干 开");
    ui->dryButton->setText("烘干 开");

    // 隐藏所有倒计时显示
    ui->uvLabel->hide();
    ui->windLabel->hide();
    ui->dryLabel->hide();
}

void MainWindow::on_upButton_clicked()
{
    // 这里实现晾衣架上升逻辑
    // 例如发送信号或调用相应函数
    qDebug() << "晾衣架上升";
}

void MainWindow::on_downButton_clicked()
{
    // 这里实现晾衣架下降逻辑
    // 例如发送信号或调用相应函数
    qDebug() << "晾衣架下降";
}

void MainWindow::updateUVTimer()
{
    if (uvTimeRemaining > 0) {
        uvTimeRemaining--;
        ui->uvLabel->setText(QString::number(uvTimeRemaining));
    } else {
        uvTimer.stop();
        ui->uvButton->setText("紫外线 开"); // 倒计时结束时重置按钮文本
        uvState = false; // 重置状态
        ui->uvLabel->hide(); // 隐藏倒计时显示
    }
}

void MainWindow::updateWindTimer()
{
    if (windTimeRemaining > 0) {
        windTimeRemaining--;
        ui->windLabel->setText(QString::number(windTimeRemaining));
    } else {
        windTimer.stop();
        ui->windButton->setText("风干 开"); // 倒计时结束时重置按钮文本
        windState = false; // 重置状态
        ui->windLabel->hide(); // 隐藏倒计时显示
    }
}

void MainWindow::updateDryTimer()
{
    if (dryTimeRemaining > 0) {
        dryTimeRemaining--;
        ui->dryLabel->setText(QString::number(dryTimeRemaining));
    } else {
        dryTimer.stop();
        ui->dryButton->setText("烘干 开"); // 倒计时结束时重置按钮文本
        dryState = false; // 重置状态
        ui->dryLabel->hide(); // 隐藏倒计时显示
    }
}

void MainWindow::on_windButtonTimer_clicked()
{
    if (dryTimeRemaining <= 31) {
        windTimeRemaining = windTimeRemaining + 4;
    }
    if (windTimeRemaining > 31) {
        windTimeRemaining = 31;
    }
}



void MainWindow::on_dryButtonTimer_clicked()
{
    if (dryTimeRemaining <= 31) {
        dryTimeRemaining = dryTimeRemaining + 4;
    }
    if (dryTimeRemaining > 31) {
        dryTimeRemaining = 31;
    }
}

