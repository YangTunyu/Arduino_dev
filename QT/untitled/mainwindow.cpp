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

    ui->setupUi(this);

    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);
    connect(ui->ledOnButton, &QPushButton::clicked, this, &MainWindow::onLEDOnButtonClicked);
    connect(ui->ledOffButton, &QPushButton::clicked, this, &MainWindow::onLEDOffButtonClicked);
    connect(tcpSocket, &QTcpSocket::readyRead, this, &MainWindow::onReadyRead);
    connect(tcpSocket, &QTcpSocket::disconnected, this, &MainWindow::onDisconnected);
    ui->setupUi(this);

    connect(ui->connectButton, &QPushButton::clicked, this, &MainWindow::onConnectButtonClicked);
    connect(ui->ledOnButton, &QPushButton::clicked, this, &MainWindow::onLEDOnButtonClicked);
    connect(ui->ledOffButton, &QPushButton::clicked, this, &MainWindow::onLEDOffButtonClicked);
    connect(tcpSocket, &QTcpSocket::readyRead, this, &MainWindow::onReadyRead);
    connect(tcpSocket, &QTcpSocket::disconnected, this, &MainWindow::onDisconnected);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onConnectButtonClicked()
{
    if (tcpSocket->state() == QAbstractSocket::UnconnectedState) {
        QString ipAddress = ui->ipAddressEdit->text();
        quint16 port = ui->portEdit->text().toUShort();
        tcpSocket->connectToHost(ipAddress, port);

        if (tcpSocket->waitForConnected(10000)) { // 增加超时时间到10秒
            qDebug() << "Connected to ESP32 on" << ipAddress << port;
        } else {
            qDebug() << "Failed to connect to ESP32 on" << ipAddress << port;
            qDebug() << tcpSocket->errorString();
        }
    } else {
        qDebug() << "Already connected or connecting";
    }
}

void MainWindow::onLEDOnButtonClicked()
{
    if (tcpSocket->state() == QAbstractSocket::ConnectedState) {
        tcpSocket->write("LED_ON\r\n");
        qDebug() << "Sent: LED_ON";
    } else {
        qDebug() << "Not connected";
    }
}

void MainWindow::onLEDOffButtonClicked()
{
    if (tcpSocket->state() == QAbstractSocket::ConnectedState) {
        tcpSocket->write("LED_OFF\r\n");
        qDebug() << "Sent: LED_OFF";
    } else {
        qDebug() << "Not connected";
    }
    if (tcpSocket->state() == QAbstractSocket::ConnectedState) {
        tcpSocket->write("LED_OFF\r\n");
        qDebug() << "Sent: LED_OFF";
    } else {
        qDebug() << "Not connected";
    }
}

void MainWindow::onReadyRead()
{
    QByteArray data = tcpSocket->readAll();
    qDebug() << "Received from ESP32:" << data;
    ui->textEdit->append(data);
    QByteArray data1 = tcpSocket->readAll();
    qDebug() << "Received from ESP32:" << data;
    ui->textEdit->append(data);
}

void MainWindow::onDisconnected()
{
    qDebug() << "Disconnected from ESP32";
    qDebug() << "Disconnected from ESP32";
}

void MainWindow::on_pushButton_clicked()
{
    if (!paused) {
        currentDirection = Up;

        updatePosition();
    }
}

void MainWindow::on_pushButton_3_clicked()
{
    if (!paused) {
        currentDirection = Down;
        updatePosition();
    }
}

void MainWindow::on_pushButton_pause_clicked()
{
    paused = !paused; // 切换暂停状态
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

    // 示例：执行某些其他操作
    if (tcpSocket->state() == QAbstractSocket::ConnectedState) {
        tcpSocket->write("LED_ON\r\n");
        qDebug() << "Sent: LED_ON";
    } else {
        qDebug() << "Not connected";
    }
}

void MainWindow::on_dial_valueChanged(int value)
{
    // 这里是处理值变化事件的逻辑代码
    // value 参数是控件当前的值，可以根据需要进行处理

    // 示例：将当前值显示在一个文本框中

    // 示例：根据值的大小改变窗口背景颜色
    if (value < 50) {
        this->setStyleSheet("background-color: lightblue;");
    } else {
        this->setStyleSheet("background-color: lightgreen;");
    }
    if (tcpSocket->state() == QAbstractSocket::UnconnectedState) {
        QString ipAddress = ui->ipAddressEdit->text();
        quint16 port = ui->portEdit->text().toUShort();
        tcpSocket->connectToHost(ipAddress, port);

        if (tcpSocket->waitForConnected(10000)) { // 增加超时时间到10秒
            qDebug() << "Connected to ESP32 on" << ipAddress << port;
        } else {
            qDebug() << "Failed to connect to ESP32 on" << ipAddress << port;
            qDebug() << tcpSocket->errorString();
        }
    } else {
        qDebug() << "Already connected or connecting";
    }

}





