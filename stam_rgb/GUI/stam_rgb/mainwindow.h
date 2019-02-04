#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "usbdevicesform.h"
#include <third-party/Qt-Color-Widgets/include/color_wheel.hpp>
#include <icdworker.h>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    color_widgets::ColorWheel* pColorWheel;
    void setup_colorwheel(color_widgets::ColorWheel* pColorWheel);
    //forms
    UsbDevicesForm* UsbDevicesDialog;

    //serial port connection
    QSerialPort *serial;


    //threads
    icdworker*      icdTxWorker;
    QThread*        icdTxWorkerThread;


private slots:

    void on_color_change(QColor);

    void IcdTx(icd::icd_header* pMsg);

    void on_ConnectPushButton_clicked();
};
#endif // MAINWINDOW_H
