#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "third-party/Qt-Color-Widgets/include/color_wheel.hpp"
#include <QtSerialPort/QtSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include "usbdevicesform.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setFixedSize(QSize(420,300));
    pColorWheel = new color_widgets::ColorWheel(this);

    setup_colorwheel(pColorWheel);



    //generate objects

    UsbDevicesDialog = new UsbDevicesForm(this);
    serial = new QSerialPort();
    icdTxWorker = new icdworker();
    icdTxWorkerThread = new QThread(this);



}

void MainWindow::setup_colorwheel(color_widgets::ColorWheel* pColorWheel)
{
    //move the color wheel to the center of the screen.
    pColorWheel->resize(200,200);
    pColorWheel->move(50,50);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_color_change(QColor color)
{
    icd::icd_header msg;

    //build basic message.

    msg.nPremble[0] = ICD_PREMBLE&0xff;
    msg.nPremble[1] = (ICD_PREMBLE&0xff00)>>8;
    msg.nPremble[2] = (ICD_PREMBLE&0xff0000)>>16;
    msg.nPremble[3] = (ICD_PREMBLE&0xff000000)>>24;




    //normalize color to 1 byte.
    msg.nRed = 255 - color.red();
    msg.nBlue = 255 - color.blue();
    msg.nGreen = 255 - color.green();
    qDebug() << "color: " << msg.nRed << msg.nBlue << msg.nGreen;




    icdTxWorker->addIcdMsgtoQueue(&msg);

}

void MainWindow::on_ConnectPushButton_clicked()
{
    //checked if thread started
    if(!UsbDevicesDialog->bThreadStarted)
    {
        UsbDevicesDialog->exec();
    }


    //setup port settings.
    serial->setPortName(UsbDevicesDialog->PortToConnectInfo.portName());
    serial->setBaudRate(115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setParity(QSerialPort::NoParity);
    serial->setStopBits(QSerialPort::OneStop);
    serial->setFlowControl(QSerialPort::NoFlowControl);

    //attempt to connect to port.
    if((serial->open(QIODevice::ReadWrite) == true)){
        //update ui logo
        ui->ConnectPushButton->setEnabled(false);

        //setup the worker.
        icdTxWorker->DoSetup(1,serial,*icdTxWorkerThread);
        icdTxWorker->moveToThread(icdTxWorkerThread);

        connect(icdTxWorker,SIGNAL(SendToTransmit(icd::icd_header*)),this,SLOT(IcdTx(icd::icd_header*))
                );


        icdTxWorkerThread->start();

        qDebug()<< "icd thread started.";
    //connect the color wheel.
        connect(pColorWheel,SIGNAL(mouseReleaseOnColor(QColor)),this,SLOT(on_color_change(QColor)));


    }

}

void MainWindow::IcdTx(icd::icd_header* pMsg)
{
    char usbTxArray[USB_MAX_MSG_SIZE];

    memcpy(&(usbTxArray[0]),static_cast<uint8_t*>(&(pMsg->nPremble[0])),sizeof(icd::icd_header));
    serial->write(usbTxArray,sizeof(icd::icd_header));


}
