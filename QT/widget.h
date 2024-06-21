#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QUrl>
class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void sendUpRequest();
    void sendDownRequest();
    void sendLightRequest();
    void sendUVRequest();
    void sendDryRequest();
    void sendFanRequest();
    void sendRequest(const QString &path);
    void turnOffLight();
    void turnOffLightAfterDelay();
    void updateStatusLabel();
    void startDryProcess();
    void stopDryProcess();
    void startFanProcess();
    void stopFanProcess();
    void startUVProcess();
    void stopUVProcess();
    void sendRequest1(const QString &path);

private:

    int remainingTime;

    QPushButton *upButton;
    QPushButton *downButton;
    QPushButton *lightButton;
    QPushButton *uvButton;
    QPushButton *fanButton;
    QPushButton *dryButton;
    QPushButton *turnOffLightButton;
    QLabel *statusLabel;
    QTimer *timer;
     QTimer *dryTimer;
     QTimer *fanTimer;
     QTimer *uvTimer;
     QTimer *dryTimer1;
};

#endif // WIDGET_H
