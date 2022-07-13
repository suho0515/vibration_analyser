#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <QDebug>
#include <QMessageBox>
#include <QtEndian>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_IMU_serialPort(new QSerialPort(this))
    , m_Motor_serialPort(new QSerialPort(this))
    , m_Motor_timer(new QTimer(this))
{
    ui->setupUi(this);

    fillIMUPortsInfo();
    fillMotorPortsInfo();

    connect(m_IMU_serialPort, &QSerialPort::readyRead, this, &MainWindow::IMU_readData);
    connect(m_IMU_serialPort, &QSerialPort::errorOccurred, this, &MainWindow::IMU_handleError);

    connect(m_Motor_serialPort, &QSerialPort::readyRead, this, &MainWindow::Motor_readData);
    connect(m_Motor_serialPort, &QSerialPort::errorOccurred, this, &MainWindow::Motor_handleError);

    connect(m_Motor_timer, SIGNAL(timeout()), this, SLOT(OnTimerCallbackFunc()));

    ui->customPlot->addGraph();
    ui->customPlot->addGraph();
    ui->customPlot->addGraph();
    ui->customPlot->addGraph();
    ui->customPlot->addGraph();
    ui->customPlot->addGraph();
    ui->customPlot->addGraph();
    ui->customPlot->addGraph();
    ui->customPlot->addGraph();
    //ui->customPlot->graph(0)->setData(x, y);
    // give the axes some labels:
    ui->customPlot->xAxis->setLabel("time(ms)");
    ui->customPlot->yAxis->setLabel("value");
    // set axes ranges, so we see all data:
    ui->customPlot->xAxis->setRange(-1, 1);
    ui->customPlot->yAxis->setRange(0, 1);
    ui->customPlot->replot();

    cnt = 0;


    // motor related
    ui->customPlotMotor->addGraph();
    ui->customPlotMotor->xAxis->setLabel("time(ms)");
    ui->customPlotMotor->yAxis->setLabel("value");
    ui->customPlotMotor->xAxis->setRange(-1, 1);
    ui->customPlotMotor->yAxis->setRange(0, 1);
    ui->customPlotMotor->replot();

    mt_cnt = 0;

    // thread
    thread = new Thread(this);
    thread->start();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::fillIMUPortsInfo()
{

    ui->cmbIMUPort->clear();

    const auto infos = QSerialPortInfo::availablePorts();
    for(const QSerialPortInfo &info : infos) {
        ui->cmbIMUPort->addItem(info.portName());
    }

    ui->textLog->append("[INFO] Available ports are added to IMU.");
}

void MainWindow::fillMotorPortsInfo()
{

    ui->cmbMotorPort->clear();

    const auto infos = QSerialPortInfo::availablePorts();
    for(const QSerialPortInfo &info : infos) {
        ui->cmbMotorPort->addItem(info.portName());
    }

    ui->textLog->append("[INFO] Available ports are added to Motor.");
}

void MainWindow::on_btnIMUConnect_clicked()
{

    m_IMU_serialPort->setPortName(ui->cmbIMUPort->currentText());
    if(ui->cmbIMUBaudrate->currentText() == "115200") {
        m_IMU_serialPort->setBaudRate(QSerialPort::Baud115200);
    }
    if(ui->cmbIMUBit->currentText() == "8") {
        m_IMU_serialPort->setDataBits(QSerialPort::Data8);
    }
    if(ui->cmbIMUParity->currentText() == "None") {
        m_IMU_serialPort->setParity(QSerialPort::NoParity);
    }
    if(ui->cmbIMUStopBit->currentText() == "1") {
        m_IMU_serialPort->setStopBits(QSerialPort::OneStop);
    }
    m_IMU_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (m_IMU_serialPort->open(QIODevice::ReadWrite)) {
        ui->btnIMUConnect->setEnabled(false);
        ui->btnIMUDisConnect->setEnabled(true);
        timer.start();
        ui->textIMUState->setText("Connected");
    }
    else {
        QMessageBox::critical(this, tr("Error"), m_IMU_serialPort->errorString());
        const auto msg = "[ERROR/IMU] " + m_IMU_serialPort->errorString();
        ui->textLog->append(msg);
        ui->textIMUState->setText("Not Connected");
    }
}

void MainWindow::on_btnIMUDisConnect_clicked()
{

    if (m_IMU_serialPort->isOpen())
        m_IMU_serialPort->close();
    ui->btnIMUConnect->setEnabled(true);
    ui->btnIMUDisConnect->setEnabled(false);
    x.clear();
    acc_x_vec.clear();
    acc_y_vec.clear();
    acc_z_vec.clear();
    rot_x_vec.clear();
    rot_y_vec.clear();
    rot_z_vec.clear();
    mag_x_vec.clear();
    mag_y_vec.clear();
    mag_z_vec.clear();

    cnt = 0;
}

void MainWindow::on_btnMotorConnect_clicked()
{

    m_Motor_serialPort->setPortName(ui->cmbMotorPort->currentText());
    if(ui->cmbMotorBaudrate->currentText() == "115200") {
        m_Motor_serialPort->setBaudRate(QSerialPort::Baud115200);
    }
    if(ui->cmbMotorBit->currentText() == "8") {
        m_Motor_serialPort->setDataBits(QSerialPort::Data8);
    }
    if(ui->cmbMotorParity->currentText() == "None") {
        m_Motor_serialPort->setParity(QSerialPort::NoParity);
    }
    if(ui->cmbMotorStopBit->currentText() == "1") {
        m_Motor_serialPort->setStopBits(QSerialPort::OneStop);
    }
    m_Motor_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    if (m_Motor_serialPort->open(QIODevice::ReadWrite)) {
        ui->btnMotorConnect->setEnabled(false);
        ui->btnMotorDisConnect->setEnabled(true);

        m_Motor_timer->start(30);
        mt_timer.start();

        ui->textMotorState->setText("Connected");
    }
    else {
        QMessageBox::critical(this, tr("Error"), m_Motor_serialPort->errorString());
        const auto msg = "[ERROR/Motor] " + m_Motor_serialPort->errorString();
        ui->textLog->append(msg);
        ui->textMotorState->setText("Not Connected");
    }
}


void MainWindow::on_btnMotorDisConnect_clicked()
{

    if (m_Motor_serialPort->isOpen())
        m_Motor_serialPort->close();
    ui->btnMotorConnect->setEnabled(true);
    ui->btnMotorDisConnect->setEnabled(false);
    m_Motor_timer->stop();

    mt_cnt = 0;

}

void MainWindow::IMU_readData()
{
    //qDebug() << "available bytes: "<< m_IMU_serialPort->bytesAvailable();
    const auto length = m_IMU_serialPort->bytesAvailable();
    if(length >= 47) {
        QByteArray data = m_IMU_serialPort->readAll();
        QByteArray hex_data = data.toHex(':');

        //qDebug() << "data.data(): " << data.data();
        //qDebug() << "hex_data.data(): " << hex_data.data();

        //qDebug() << "data: " << data;
        //qDebug() << "hex_data: " << hex_data;
        //qDebug() << "hex_data length: " << hex_data.size();

        QByteArrayList ba_list = hex_data.split(':');
        //qDebug() << "ba_list: " << ba_list;

        if(ba_list[0] == "55"
                && ba_list[1] == "55"
                && ba_list[2] == "7a"
                && ba_list[3] == "31"
                && ba_list[4] == "28") {

            QByteArray buffer;
            QByteArray ba_1;
            QByteArray ba_2;
            QByteArray ba_3;
            QByteArray ba_4;
            quint32 localEndian;

            // System Time Parsing
            ba_1 = QByteArray::fromHex(ba_list[5]);
            ba_2 = QByteArray::fromHex(ba_list[6]);
            ba_3 = QByteArray::fromHex(ba_list[7]);
            ba_4 = QByteArray::fromHex(ba_list[8]);

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            time = qFromLittleEndian<quint32>(buffer.data());

            buffer.clear();

            //qDebug() << "time: " << time;



            // Acceleration X Parsing

            ba_1 = QByteArray::fromHex(ba_list[9]);
            ba_2 = QByteArray::fromHex(ba_list[10]);
            ba_3 = QByteArray::fromHex(ba_list[11]);
            ba_4 = QByteArray::fromHex(ba_list[12]);

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            localEndian = qFromLittleEndian<quint32>(buffer.data());
            acc_x = *reinterpret_cast<float *>(&localEndian);

            buffer.clear();

            //qDebug() << "acc_x: " << acc_x;



            // Acceleration Y Parsing

            ba_1 = QByteArray::fromHex(ba_list[13]);
            ba_2 = QByteArray::fromHex(ba_list[14]);
            ba_3 = QByteArray::fromHex(ba_list[15]);
            ba_4 = QByteArray::fromHex(ba_list[16]);

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            localEndian = qFromLittleEndian<quint32>(buffer.data());
            acc_y = *reinterpret_cast<float *>(&localEndian);

            buffer.clear();

            //qDebug() << "acc_y: " << acc_y;



            // Acceleration Z Parsing

            ba_1 = QByteArray::fromHex(ba_list[17]);
            ba_2 = QByteArray::fromHex(ba_list[18]);
            ba_3 = QByteArray::fromHex(ba_list[19]);
            ba_4 = QByteArray::fromHex(ba_list[20]);

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            localEndian = qFromLittleEndian<quint32>(buffer.data());
            acc_z = *reinterpret_cast<float *>(&localEndian);

            buffer.clear();

            //qDebug() << "acc_z: " << acc_z;



            // Rotation X Parsing

            ba_1 = QByteArray::fromHex(ba_list[21]);
            ba_2 = QByteArray::fromHex(ba_list[22]);
            ba_3 = QByteArray::fromHex(ba_list[23]);
            ba_4 = QByteArray::fromHex(ba_list[24]);

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            localEndian = qFromLittleEndian<quint32>(buffer.data());
            rot_x = *reinterpret_cast<float *>(&localEndian);

            buffer.clear();

            //qDebug() << "rot_x: " << rot_x;



            // Rotation Y Parsing

            ba_1 = QByteArray::fromHex(ba_list[25]);
            ba_2 = QByteArray::fromHex(ba_list[26]);
            ba_3 = QByteArray::fromHex(ba_list[27]);
            ba_4 = QByteArray::fromHex(ba_list[28]);

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            localEndian = qFromLittleEndian<quint32>(buffer.data());
            rot_y = *reinterpret_cast<float *>(&localEndian);

            buffer.clear();

            //qDebug() << "rot_y: " << rot_y;



            // Rotation Z Parsing

            ba_1 = QByteArray::fromHex(ba_list[29]);
            ba_2 = QByteArray::fromHex(ba_list[30]);
            ba_3 = QByteArray::fromHex(ba_list[31]);
            ba_4 = QByteArray::fromHex(ba_list[32]);

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            localEndian = qFromLittleEndian<quint32>(buffer.data());
            rot_z = *reinterpret_cast<float *>(&localEndian);

            buffer.clear();

            //qDebug() << "rot_z: " << rot_z;



            // Magnetic X Parsing

            ba_1 = QByteArray::fromHex(ba_list[33]);
            ba_2 = QByteArray::fromHex(ba_list[34]);
            ba_3 = QByteArray::fromHex(ba_list[35]);
            ba_4 = QByteArray::fromHex(ba_list[36]);

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            localEndian = qFromLittleEndian<quint32>(buffer.data());
            mag_x = *reinterpret_cast<float *>(&localEndian);

            buffer.clear();

            //qDebug() << "mag_x: " << mag_x;



            // Magnetic Y Parsing

            ba_1 = QByteArray::fromHex(ba_list[37]);
            ba_2 = QByteArray::fromHex(ba_list[38]);
            ba_3 = QByteArray::fromHex(ba_list[39]);
            ba_4 = QByteArray::fromHex(ba_list[40]);

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            localEndian = qFromLittleEndian<quint32>(buffer.data());
            mag_y = *reinterpret_cast<float *>(&localEndian);

            buffer.clear();

            //qDebug() << "mag_y: " << mag_y;



            // Magnetic Z Parsing

            ba_1 = QByteArray::fromHex(ba_list[41]);
            ba_2 = QByteArray::fromHex(ba_list[42]);
            ba_3 = QByteArray::fromHex(ba_list[43]);
            ba_4 = QByteArray::fromHex(ba_list[44]);

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            localEndian = qFromLittleEndian<quint32>(buffer.data());
            mag_z = *reinterpret_cast<float *>(&localEndian);

            buffer.clear();

            //qDebug() << "mag_z: " << mag_z;


            ui->textAccX->setText(QString::number(acc_x));
            ui->textAccY->setText(QString::number(acc_y));
            ui->textAccZ->setText(QString::number(acc_z));
            ui->textRotX->setText(QString::number(rot_x));
            ui->textRotY->setText(QString::number(rot_y));
            ui->textRotZ->setText(QString::number(rot_z));
            ui->textMagX->setText(QString::number(mag_x));
            ui->textMagY->setText(QString::number(mag_y));
            ui->textMagZ->setText(QString::number(mag_z));


            //qDebug() << timer.elapsed(); //msec
            x.append(timer.elapsed());
            //y.append(rot_z);
            acc_x_vec.append(acc_x);
            acc_y_vec.append(acc_y);
            acc_z_vec.append(acc_z);
            rot_x_vec.append(rot_x);
            rot_y_vec.append(rot_y);
            rot_z_vec.append(rot_z);
            mag_x_vec.append(mag_x);
            mag_y_vec.append(mag_y);
            mag_z_vec.append(mag_z);

            cnt = cnt + 1;

            double hz = (static_cast<double>(cnt))/((static_cast<double>(timer.elapsed()))/1000.0);
            double ms = 1000.0/hz;

            ui->textCount->setText(QString::number(cnt));
            ui->textFrequency->setText(QString::number(hz));
            ui->textPeriod->setText(QString::number(ms));

            //qDebug() << ((double)cnt)/(((double)timer.elapsed())/1000.0);

            //qDebug() << "Drawing chart";
            // assign data to it:
            ui->customPlot->graph(0)->setData(x, acc_x_vec);
            ui->customPlot->graph(1)->setData(x, acc_y_vec);
            ui->customPlot->graph(2)->setData(x, acc_z_vec);
            ui->customPlot->graph(3)->setData(x, rot_x_vec);
            ui->customPlot->graph(4)->setData(x, rot_y_vec);
            ui->customPlot->graph(5)->setData(x, rot_z_vec);
            ui->customPlot->graph(6)->setData(x, mag_x_vec);
            ui->customPlot->graph(7)->setData(x, mag_y_vec);
            ui->customPlot->graph(8)->setData(x, mag_z_vec);
            // set axes ranges, so we see all data:
            ui->customPlot->xAxis->setRange(0, timer.elapsed());
            ui->customPlot->yAxis->setRange(-360, 360);
            ui->customPlot->replot();

        }
    }
}


void MainWindow::Motor_readData()
{
    const auto length = m_Motor_serialPort->bytesAvailable();
    //qDebug() << length;
    if(length >= 20) {

        QByteArray data = m_Motor_serialPort->readAll();
        //qDebug() << data;

        if(data[0]=='Q' && data[1]=='P') {
            QByteArray buffer;
            QByteArray ba_1;
            QByteArray ba_2;
            QByteArray ba_3;
            QByteArray ba_4;

            //for(int i=2; i< 10; i++) {
            //    buffer.append(data[i]);
            //}

            buffer.clear();

            buffer.append(data[2]);
            buffer.append(data[3]);

            ba_1 = QByteArray::fromHex(buffer);

            buffer.clear();

            buffer.append(data[4]);
            buffer.append(data[5]);

            ba_2 = QByteArray::fromHex(buffer);

            buffer.clear();

            buffer.append(data[6]);
            buffer.append(data[7]);

            ba_3 = QByteArray::fromHex(buffer);

            buffer.clear();

            buffer.append(data[8]);
            buffer.append(data[9]);

            ba_4 = QByteArray::fromHex(buffer);

            buffer.clear();

            buffer.append(ba_1);
            buffer.append(ba_2);
            buffer.append(ba_3);
            buffer.append(ba_4);

            position = qFromBigEndian<quint32>(buffer.data());
            x_motor.append(mt_timer.elapsed());
            y_motor.append(position);

            buffer.clear();

            //qDebug() << position;

            mt_cnt = mt_cnt + 1;

            double hz = (static_cast<double>(mt_cnt))/((static_cast<double>(mt_timer.elapsed()))/1000.0);
            double ms = 1000.0/hz;

            ui->textMotorCount->setText(QString::number(mt_cnt));
            ui->textMotorFrequency->setText(QString::number(hz));
            ui->textMotorPeriod->setText(QString::number(ms));

            // assign data to it:
            ui->customPlotMotor->graph(0)->setData(x_motor, y_motor);
            // set axes ranges, so we see all data:
            ui->customPlotMotor->xAxis->setRange(0, mt_timer.elapsed());
            ui->customPlotMotor->yAxis->setRange(4900000, 5100000);
            ui->customPlotMotor->replot();
        }

    }
}

void MainWindow::IMU_handleError(QSerialPort::SerialPortError error)
{

    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), m_IMU_serialPort->errorString());
        on_btnIMUDisConnect_clicked();
    }

}

void MainWindow::Motor_handleError(QSerialPort::SerialPortError error)
{

    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), m_Motor_serialPort->errorString());
        on_btnMotorDisConnect_clicked();
    }

}

void MainWindow::OnTimerCallbackFunc()
{
    QTime time = QTime::currentTime();
    QString time_text = time.toString("hh : mm : ss");
    ui->textLog->setText(time_text);
    //ui->textLog->setText(time_text);
    m_Motor_serialPort->write("QP;");
}

