#include "widget.h"
#include<QTimer>
#include <QInputDialog>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
{
    // 创建按钮
//    upButton = new QPushButton("上升");
//    downButton = new QPushButton("下降");
    lightButton = new QPushButton("照明 关闭");
    uvButton = new QPushButton("紫外线杀菌 关闭");
    fanButton = new QPushButton("风干 关闭");
    dryButton = new QPushButton("烘干 关闭");
    turnOffLightButton = new QPushButton("15分钟后关灯");

    // 创建标签
    statusLabel = new QLabel("状态：空闲");

    // 创建按钮组布局
    QVBoxLayout *buttonLayout = new QVBoxLayout;
//    buttonLayout->addWidget(upButton);
//    buttonLayout->addWidget(downButton);
    buttonLayout->addWidget(lightButton);
    buttonLayout->addWidget(uvButton);
    buttonLayout->addWidget(dryButton);
    buttonLayout->addWidget(fanButton);
    buttonLayout->addWidget(turnOffLightButton);

    // 创建主布局
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(buttonLayout);
    mainLayout->addWidget(statusLabel);

    // 设置布局
    setLayout(mainLayout);
    setWindowTitle("智能晾衣架控制");

    // 连接按钮点击信号与槽函数
//    connect(upButton, &QPushButton::clicked, this, &Widget::sendUpRequest);
//    connect(downButton, &QPushButton::clicked, this, &Widget::sendDownRequest);
    connect(lightButton, &QPushButton::clicked, this, &Widget::sendLightRequest);
    connect(uvButton, &QPushButton::clicked, this, &Widget::sendUVRequest);
    connect(dryButton, &QPushButton::clicked, this, &Widget::sendDryRequest);
    connect(fanButton, &QPushButton::clicked, this, &Widget::sendFanRequest);
    connect(turnOffLightButton, &QPushButton::clicked, this, &Widget::turnOffLightAfterDelay);

    // 初始化定时器
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Widget::turnOffLight);
    dryTimer = new QTimer(this);
    dryTimer1 = new QTimer(this);
    connect(dryTimer, &QTimer::timeout, this, &Widget::stopDryProcess);
    fanTimer = new QTimer(this);
    connect(fanTimer, &QTimer::timeout, this, &Widget::stopFanProcess);
    uvTimer = new QTimer(this);
    connect(uvTimer, &QTimer::timeout, this, &Widget::stopUVProcess);

}

Widget::~Widget()
{
}

void Widget::sendUpRequest()
{
    sendRequest1("/up");
}

void Widget::sendDownRequest()
{
    sendRequest1("/down");
}

void Widget::sendLightRequest()
{
    if (lightButton->text() == "照明 关闭") {
        sendRequest("/light/on");
        lightButton->setText("照明 开启");
    } else {
        sendRequest("/light/off");
        lightButton->setText("照明 关闭");
    }
}

void Widget::startUVProcess()
{
    bool ok;
    sendRequest("/uv/off");
    int minutes = QInputDialog::getInt(this, tr("设置紫外线时间"), tr("紫外线时间（分钟）:"), 10, 1, 60, 1, &ok);
    if (ok) {
        // 如果有紫外线定时器正在运行，则先停止它
        if (uvTimer->isActive()) {
            uvTimer->stop();
        }

        // 发送烘干命令
        uvButton->setText("紫外线杀菌 关闭");
        statusLabel->setText("状态：紫外线杀菌 开启");
        sendRequest("/uv/on");
        remainingTime = minutes * 60; // 转换为秒
        uvTimer->start(1000 * 60 * minutes); // 启动定时器，时间到后关闭风干功能
    }
}
void Widget::stopUVProcess()
{
    // 停止紫外线杀菌功能
    uvButton->setText("紫外线杀菌 开启");
    statusLabel->setText("状态：紫外线杀菌 关闭");
    sendRequest("/uv/off");
    uvTimer->stop();
}


void Widget::sendUVRequest()
{
    if (uvButton->text() == "紫外线杀菌 关闭") {
        startUVProcess();
    } else {
        sendRequest("/uv/off");
    }
}
void Widget::startDryProcess()
{
    bool ok;
    sendRequest("/dry/off");
    int minutes = QInputDialog::getInt(this, tr("设置烘干时间"), tr("烘干时间（分钟）:"), 10, 1, 60, 1, &ok);
    if (ok) {
        // 如果有烘干定时器正在运行，则先停止它
        if (dryTimer->isActive()) {
            dryTimer->stop();
        }

        // 发送烘干命令
        dryButton->setText("烘干 关闭");
        fanButton->setText("风干 开启");
        statusLabel->setText("状态：烘干 开启");
        sendRequest("/dry/on");
        remainingTime = minutes * 60; // 转换为秒
        dryTimer->start(1000 * 60 * minutes); // 启动定时器，时间到后关闭风干功能
        //dryTimer1->start(1000); // 启动定时器，每秒触发一次timeout()信号
        //connect(dryTimer1, &QTimer::timeout, this, &Widget::updateStatusLabel); // 连接定时器的timeout()信号到updateStatusLabel()函数

    }
}
void Widget::stopDryProcess()
{
    // 停止烘干功能
    dryButton->setText("烘干 开启");
    statusLabel->setText("状态：烘干 关闭");
    sendRequest("/dry/off");
    dryTimer->stop();
}

void Widget::updateStatusLabel()
{
    if (remainingTime > 0) {
        remainingTime--;
        int minutes = remainingTime / 60;
        int seconds = remainingTime % 60;
        QString timeString = QString("%1分%2秒").arg(minutes).arg(seconds, 2, 10, QChar('0'));
        statusLabel->setText("状态：烘干 开启，剩余时间：" + timeString);
    } else {
        dryTimer->stop();
        // 时间耗尽，关闭烘干功能
        dryButton->setText("烘干 开启");
        statusLabel->setText("状态：烘干 关闭");
    }
}

void Widget::sendDryRequest()
{
    if (dryButton->text() == "烘干 关闭") {
        startDryProcess();
    } else {
        sendRequest("/dry/off");
    }
}
void Widget::startFanProcess()
{
    bool ok;
    sendRequest("/fan/off");
    int minutes = QInputDialog::getInt(this, tr("设置风干时间"), tr("风干时间（分钟）:"), 10, 1, 60, 1, &ok);
    if (ok) {
        // 发送风干命令
        fanButton->setText("风干 开启");
        statusLabel->setText("状态：风干 开启");
        sendRequest("/fan/on");
        remainingTime = minutes * 60; // 转换为秒
        fanTimer->start(1000 * 60 * minutes); // 启动定时器，时间到后关闭风干功能
    }
}
void Widget::stopFanProcess()
{
    // 停止风干功能
    fanButton->setText("风干 开启");
    statusLabel->setText("状态：风干 关闭");
    sendRequest("/fan/off");
    fanTimer->stop();
}
void Widget::sendFanRequest()
{
    if (fanButton->text() == "风干 关闭") {
        startFanProcess();
    } else {
        sendRequest("/fan/off");
    }
}



void Widget::sendRequest1(const QString &path)
{
    QTcpSocket *socket = new QTcpSocket(this);
    socket->connectToHost("192.168.217.149", 80); // 替换为ESP32服务器的IP地址和端口

    if (socket->waitForConnected()) {
        QString request = "GET " + path + " HTTP/1.1\r\n"
                          "Host: 192.168.217.149\r\n" // 替换为ESP32服务器的IP地址
                          "Connection: close\r\n\r\n";

        socket->write(request.toUtf8());
        socket->waitForBytesWritten();

        // 读取服务器响应
        QByteArray responseData;
        while (socket->waitForReadyRead()) {
            responseData.append(socket->readAll());
        }

        // 处理服务器返回的数据
        qDebug() << "Server Response:" << responseData;

        // 解析服务器返回的数据
        QString responseString = QString::fromUtf8(responseData);
        if (responseString.contains("up")) {
            statusLabel->setText("状态：上升中");
        } else if (responseString.contains("down")) {
            statusLabel->setText("状态：下降中");
        }

        socket->close();
        } else {
            qDebug() << "Failed to connect to server:" << socket->errorString();
        }
socket->deleteLater();
}
void Widget::sendRequest(const QString &path)
{
    QTcpSocket *socket = new QTcpSocket(this);
    socket->connectToHost("192.168.217.149", 80); // 替换为ESP32服务器的IP地址和端口

    if (socket->waitForConnected()) {
        QString request = "GET " + path + " HTTP/1.1\r\n"
                          "Host: 192.168.217.149\r\n" // 替换为ESP32服务器的IP地址
                          "Connection: close\r\n\r\n";

        socket->write(request.toUtf8());
        socket->waitForBytesWritten();

        // 读取服务器响应
        QByteArray responseData;
        while (socket->waitForReadyRead()) {
            responseData.append(socket->readAll());
        }

        // 处理服务器返回的数据
        qDebug() << "Server Response:" << responseData;

        // 解析服务器返回的数据
        QString responseString = QString::fromUtf8(responseData);
        if (responseString.contains("up")) {
            statusLabel->setText("状态：上升中");
        } else if (responseString.contains("down")) {
            statusLabel->setText("状态：下降中");
        } else if (responseString.contains("light/on")) {
            lightButton->setText("照明 开启");
            statusLabel->setText("状态：照明 开启");
        } else if (responseString.contains("light/off")) {
            lightButton->setText("照明 关闭");
            statusLabel->setText("状态：照明 关闭");
        } else if (responseString.contains("uv/on")) {
            uvButton->setText("紫外线杀菌 开启");
            statusLabel->setText("状态：紫外线杀菌 开启");
        } else if (responseString.contains("uv/off")) {
            uvButton->setText("紫外线杀菌 关闭");
            statusLabel->setText("状态：紫外线杀菌 关闭");
        } else if (responseString.contains("fan/on")) {
            fanButton->setText("风干 开启");
            statusLabel->setText("状态：风干 开启");
        } else if (responseString.contains("fan/off")) {
            fanButton->setText("风干 关闭");
            statusLabel->setText("状态：风干 关闭");
        }else if (responseString.contains("dry/on")) {
            dryButton->setText("烘干 开启");
            statusLabel->setText("状态：烘干 开启");
        } else if (responseString.contains("dry/off")) {
            fanButton->setText("风干 关闭");
            dryButton->setText("烘干 关闭");
            statusLabel->setText("状态：烘干 关闭");
        }
        else {
            statusLabel->setText("状态：未知");
        }

        socket->close();
    } else {
        qDebug() << "Failed to connect to server:" << socket->errorString();
        // 处理连接失败
    }

    socket->deleteLater();
}


void Widget::turnOffLightAfterDelay()
{
    // 启动定时器，15分钟后关灯
    sendRequest("/light/on");
    lightButton->setText("照明 开启");
    timer->start(15 *60 * 1000); // 15分钟 = 15 * 60秒 * 1000毫秒
    statusLabel->setText("状态：等待15分钟后关灯");
}

void Widget::turnOffLight()
{
    // 发送关灯指令
    sendRequest("/light/off");
    lightButton->setText("照明 关闭");
    statusLabel->setText("状态：照明 关闭");
    // 停止定时器
    timer->stop();
}
