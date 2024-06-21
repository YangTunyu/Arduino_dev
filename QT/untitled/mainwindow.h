#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class QTimer;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_pause_clicked();
    void updatePosition();

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    int position;
    enum Direction { None, Up, Down }; // 枚举类型放在这里声明
    Direction currentDirection;
    bool paused;
};

#endif // MAINWINDOW_H
