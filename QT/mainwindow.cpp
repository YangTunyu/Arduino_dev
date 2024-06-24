#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QNetworkRequest>
#include <QNetworkReply>

#define ESP32_IP_ADDRESS "192.168.43.198"

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
    , networkManager(new QNetworkAccessManager(this))
{
    ui->setupUi(this);

    connect(&uvTimer, &QTimer::timeout, this, &MainWindow::updateUVTimer);
    connect(&windTimer, &QTimer::timeout, this, &MainWindow::updateWindTimer);
    connect(&dryTimer, &QTimer::timeout, this, &MainWindow::updateDryTimer);
    connect(networkManager, &QNetworkAccessManager::finished, this, &MainWindow::handleNetworkReply);

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

void MainWindow::sendRequest(const QString &url)
{
    QNetworkRequest request;
    request.setUrl(QUrl(url));
    networkManager->get(request);
}

void MainWindow::handleNetworkReply(QNetworkReply *reply)
{
    if (reply->error() == QNetworkReply::NoError) {
        QByteArray response = reply->readAll();
        qDebug() << "Response from ESP32:" << response;
    } else {
        qDebug() << "Error:" << reply->errorString();
    }
    reply->deleteLater();
}

void MainWindow::on_lightButton_clicked()
{
    lightState = !lightState; // 切换状态
    if (lightState) {
        ui->lightButton->setText("照明 关");
        sendRequest("http://" ESP32_IP_ADDRESS ":80/light_on");
    } else {
        ui->lightButton->setText("照明 开");
        sendRequest("http://" ESP32_IP_ADDRESS ":80/light_off");
    }
}

void MainWindow::on_uvButton_clicked()
{
    uvState = !uvState; // 切换状态
    if (uvState) {
        uvTimeRemaining = 15; // 重置倒计时
        ui->uvLabel->setText(QString::number(uvTimeRemaining));
        ui->uvLabel->show();
        uvTimer.start(1000); // 每秒更新一次
        ui->uvButton->setText("紫外线 关");
        sendRequest("http://" ESP32_IP_ADDRESS ":80/uv_on");
    } else {
        uvTimer.stop();
        ui->uvLabel->hide();
        ui->uvButton->setText("紫外线 开");
        sendRequest("http://" ESP32_IP_ADDRESS ":80/uv_off");
    }
}

void MainWindow::on_windButton_clicked()
{
    windState = !windState; // 切换风干的状态
    if (windState) {
        windTimeRemaining = 15; // 重置倒计时
        ui->windLabel->setText(QString::number(windTimeRemaining));
        ui->windLabel->show();
        windTimer.start(1000); // 每秒更新一次
        ui->windButton->setText("风干 关");
        sendRequest("http://" ESP32_IP_ADDRESS ":80/wind_on");
    } else {
        windTimer.stop();
        ui->windLabel->hide();
        ui->windButton->setText("风干 开");
        sendRequest("http://" ESP32_IP_ADDRESS ":80/wind_off");
    }
}

void MainWindow::on_dryButton_clicked()
{
    dryState = !dryState; // 切换状态
    if (dryState) {
        dryTimeRemaining = 15; // 重置倒计时
        ui->dryLabel->setText(QString::number(dryTimeRemaining));
        ui->dryLabel->show();
        dryTimer.start(1000); // 每秒更新一次
        ui->dryButton->setText("烘干 关");
        sendRequest("http://" ESP32_IP_ADDRESS ":80/dry_on");
    } else {
        dryTimer.stop();
        ui->dryLabel->hide();
        ui->dryButton->setText("烘干 开");
        sendRequest("http://" ESP32_IP_ADDRESS ":80/dry_off");
    }
}

void MainWindow::on_stopButton_clicked()
{
    uvTimer.stop();
    windTimer.stop();
    dryTimer.stop();

    uvState = false;
    windState = false;
    dryState = false;
    ui->uvButton->setText("紫外线 开");
    ui->windButton->setText("风干 开");
    ui->dryButton->setText("烘干 开");

    ui->uvLabel->hide();
    ui->windLabel->hide();
    ui->dryLabel->hide();

    sendRequest("http://" ESP32_IP_ADDRESS ":80/stop_all");
}

void MainWindow::on_upButton_clicked()
{
    qDebug() << "晾衣架上升";
    sendRequest("http://" ESP32_IP_ADDRESS ":80/up");
}

void MainWindow::on_downButton_clicked()
{
    qDebug() << "晾衣架下降";
    sendRequest("http://" ESP32_IP_ADDRESS ":80/down");
}

void MainWindow::updateUVTimer()
{
    if (uvTimeRemaining > 0) {
        uvTimeRemaining--;
        ui->uvLabel->setText(QString::number(uvTimeRemaining));
    } else {
        uvTimer.stop();
        ui->uvButton->setText("紫外线 开");
        uvState = false;
        ui->uvLabel->hide();
        sendRequest("http://" ESP32_IP_ADDRESS ":80/uv_off");
    }
}

void MainWindow::updateWindTimer()
{
    if (windTimeRemaining > 0) {
        windTimeRemaining--;
        ui->windLabel->setText(QString::number(windTimeRemaining));
    } else {
        windTimer.stop();
        ui->windButton->setText("风干 开");
        windState = false;
        ui->windLabel->hide();
        sendRequest("http://" ESP32_IP_ADDRESS ":80/wind_off");
    }
}

void MainWindow::updateDryTimer()
{
    if (dryTimeRemaining > 0) {
        dryTimeRemaining--;
        ui->dryLabel->setText(QString::number(dryTimeRemaining));
    } else {
        dryTimer.stop();
        ui->dryButton->setText("烘干 开");
        dryState = false;
        ui->dryLabel->hide();
        sendRequest("http://" ESP32_IP_ADDRESS ":80/dry_off");
    }
}

void MainWindow::on_windButtonTimer_clicked()
{
    windTimeRemaining += 3; // 每次增加3秒
    if (windTimeRemaining > 30) { // 上限30秒
        windTimeRemaining = 30;
    }
    sendRequest("http://" ESP32_IP_ADDRESS ":80/set_fan_duration?duration=" + QString::number(windTimeRemaining));
}

void MainWindow::on_dryButtonTimer_clicked()
{
    dryTimeRemaining += 3; // 每次增加3秒
    if (dryTimeRemaining > 30) { // 上限30秒
        dryTimeRemaining = 30;
    }
    sendRequest("http://" ESP32_IP_ADDRESS ":80/set_dry_duration?duration=" + QString::number(dryTimeRemaining));
}
