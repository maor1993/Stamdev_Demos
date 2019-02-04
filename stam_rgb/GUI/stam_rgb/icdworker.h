#ifndef ICDWORKER_H
#define ICDWORKER_H

#include <QObject>
#include <QSerialPort>
#include "tempostructs.h"
#include <QQueue>

class icdworker : public QObject
{
    Q_OBJECT
public:
    explicit icdworker(QObject *parent = nullptr);
    void DoSetup(bool bWorkerType,QSerialPort* serial,QThread &cThread);
    void addIcdMsgtoQueue(icd::icd_header* pMsg);
    void USBDisconnect();
    int USBConnect(QSerialPortInfo* SerialInfo);

signals:
    void SendToTransmit(icd::icd_header* pMsg);
public slots:
    void DoWorkTX();
private:
    QSerialPort* serial;
    QQueue<icd::icd_header>* IcdTxQueue;
};

#endif // ICDWORKER_H
