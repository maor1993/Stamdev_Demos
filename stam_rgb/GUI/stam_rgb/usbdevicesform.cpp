#include "usbdevicesform.h"
#include "ui_usbdevicesform.h"
#include <QtCore>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

UsbDevicesForm::UsbDevicesForm(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::UsbDevicesForm)
{
    ui->setupUi(this);
    bThreadStarted = false;
    CollectDevices();
}

UsbDevicesForm::~UsbDevicesForm()
{
    delete ui;
    bThreadStarted = true;
}

void UsbDevicesForm::CollectDevices()
{
    //get all connected devices.
     AvilablePortsList = QSerialPortInfo::availablePorts();
     foreach(QSerialPortInfo l,AvilablePortsList)
     {
        ui->UsbDevicesList->addItem(l.portName());
     }
}

void UsbDevicesForm::on_UsbConnectButton_clicked()
{


    //connect to selected from list.
      QSerialPort PortToConnect;

       PortToConnectInfo = AvilablePortsList.at(ui->UsbDevicesList->currentIndex().row());


       //setup settings.


        this->close();

}
