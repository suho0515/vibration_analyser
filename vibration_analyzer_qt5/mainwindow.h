#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "thread.h"

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QElapsedTimer>
#include <QTimer>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void IMU_readData();
    void IMU_handleError(QSerialPort::SerialPortError error);

    void Motor_readData();
    void Motor_handleError(QSerialPort::SerialPortError error);

    void on_btnIMUConnect_clicked();
    void on_btnIMUDisConnect_clicked();

    void on_btnMotorConnect_clicked();
    void on_btnMotorDisConnect_clicked();

    void OnTimerCallbackFunc();


private:
    Ui::MainWindow *ui;

    void fillIMUPortsInfo();
    void fillMotorPortsInfo();

    QSerialPort* m_IMU_serialPort = nullptr;
    QSerialPort* m_Motor_serialPort = nullptr;

    bool msgFlag = false;
    QByteArray global_data;

    uint time;
    float acc_x;
    float acc_y;
    float acc_z;
    float rot_x;
    float rot_y;
    float rot_z;
    float mag_x;
    float mag_y;
    float mag_z;

    QVector<double> x;
    QVector<double> acc_x_vec, acc_y_vec, acc_z_vec, rot_x_vec, rot_y_vec, rot_z_vec, mag_x_vec, mag_y_vec, mag_z_vec;
    //QVector<double> y;

    int cnt;
    QElapsedTimer timer;

    // Motor related
    QTimer *m_Motor_timer = nullptr;

    QElapsedTimer mt_timer;
    unsigned long long position;

    int mt_cnt = 0;

    QVector<double> x_motor;
    QVector<double> y_motor;

    // thread
private:
    Thread *thread;
};



#endif // MAINWINDOW_H
