#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "thread.h"
#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>


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
    void IMU_readData();
    void IMU_handleError(QSerialPort::SerialPortError error);

    void Motor_readData();
    void Motor_handleError(QSerialPort::SerialPortError error);

    void on_btnIMUConnect_clicked();
    void on_btnIMUDisConnect_clicked();

    void on_btnMotorConnect_clicked();
    void on_btnMotorDisConnect_clicked();

private:
    Ui::MainWindow *ui;

    Thread *thread;

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


private slots:
    void Receive(int data);
};
#endif // MAINWINDOW_H
