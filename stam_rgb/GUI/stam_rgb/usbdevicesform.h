#ifndef USBDEVICESFORM_H
#define USBDEVICESFORM_H

#include <QDialog>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
namespace Ui {
class UsbDevicesForm;
}

class UsbDevicesForm : public QDialog
{
    Q_OBJECT

public:
    explicit UsbDevicesForm(QWidget *parent = 0);
    ~UsbDevicesForm();
    bool bThreadStarted;
    QSerialPortInfo PortToConnectInfo;
private slots:
    void on_UsbConnectButton_clicked();

private:
    Ui::UsbDevicesForm *ui;
    void CollectDevices();
    QList<QSerialPortInfo> AvilablePortsList;

};

#endif // USBDEVICESFORM_H
