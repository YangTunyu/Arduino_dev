#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QNetworkAccessManager>
#include <QNetworkReply>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_lightButton_clicked();
    void on_uvButton_clicked();
    void on_windButton_clicked();
    void on_dryButton_clicked();
    void on_stopButton_clicked();
    void on_upButton_clicked();
    void on_downButton_clicked();
    void updateUVTimer();
    void updateWindTimer();
    void updateDryTimer();
    void on_windButtonTimer_clicked();
    void on_dryButtonTimer_clicked();
    void handleNetworkReply(QNetworkReply *reply);

private:
    Ui::MainWindow *ui;
    QTimer uvTimer;
    QTimer windTimer;
    QTimer dryTimer;
    int uvTimeRemaining;
    int windTimeRemaining;
    int dryTimeRemaining;
    bool lightState;
    bool uvState;
    bool windState;
    bool dryState;
    QNetworkAccessManager *networkManager;

    void sendRequest(const QString &url);
};

#endif // MAINWINDOW_H







