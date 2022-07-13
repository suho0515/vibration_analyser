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
{
    ui->setupUi(this);

    thread = new Thread(this);
    thread->start();

    fillIMUPortsInfo();
    fillMotorPortsInfo();

    connect(m_IMU_serialPort, &QSerialPort::readyRead, this, &MainWindow::IMU_readData);
    connect(m_IMU_serialPort, &QSerialPort::errorOccurred, this, &MainWindow::IMU_handleError);

    connect(m_Motor_serialPort, &QSerialPort::readyRead, this, &MainWindow::Motor_readData);
    connect(m_Motor_serialPort, &QSerialPort::errorOccurred, this, &MainWindow::Motor_handleError);

    //connect(thread, SIGNAL(Send(int)), this, SLOT(Receive(int)));

    // generate some data:
    /*
    QVector<double> x(101), y(101); // initialize with entries 0..100
    for (int i=0; i<101; ++i)
    {
      x[i] = i/50.0 - 1; // x goes from -1 to 1
      y[i] = x[i]*x[i]; // let's plot a quadratic function
    }
    // create graph and assign data to it:
    ui->customPlot->addGraph();
    ui->customPlot->graph(0)->setData(x, y);
    // give the axes some labels:
    ui->customPlot->xAxis->setLabel("x");
    ui->customPlot->yAxis->setLabel("y");
    // set axes ranges, so we see all data:
    ui->customPlot->xAxis->setRange(-1, 1);
    ui->customPlot->yAxis->setRange(0, 1);
    ui->customPlot->replot();
    */
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

}

void MainWindow::IMU_readData()
{
    //qDebug() << "available bytes: "<< m_IMU_serialPort->bytesAvailable();
    const auto length = m_IMU_serialPort->bytesAvailable();
    if(length == 47) {
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

        }
    }
}


void MainWindow::Motor_readData()
{

    QByteArray data = m_Motor_serialPort->readAll();
    ui->textLog->append(QString::number(data.size()));

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

void MainWindow::Receive(int data)
{
    qDebug() << data;
}
