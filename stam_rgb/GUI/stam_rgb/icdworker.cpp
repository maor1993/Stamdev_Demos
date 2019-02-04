#include "icdworker.h"
#include "tempostructs.h"
#include <QtCore>
#include <QSerialPort>
#include "mainwindow.h"
#include "tempostructs.h"

icdworker::icdworker(QObject *parent) : QObject(parent)
{
    IcdTxQueue = new QQueue<icd::icd_header>();
}

int icdworker::USBConnect(QSerialPortInfo* SerialInfo)
{
    return 0;
}
void icdworker::USBDisconnect()
{
    serial->close();
}


void icdworker::DoSetup(bool bWorkerType,QSerialPort* serial,QThread &cThread)
{
    this->serial = serial;


    if(bWorkerType) //tx
    {
        connect(&cThread,SIGNAL(started()),this,SLOT(DoWorkTX()));
    }




}


void icdworker::DoWorkTX()
{
    icd::icd_header msg;

    while(1)
    {
        if(!IcdTxQueue->isEmpty())
        {
            msg = IcdTxQueue->dequeue();

            SendToTransmit(&msg);
        }
        QThread::msleep(100);

    }
}



void icdworker::addIcdMsgtoQueue(icd::icd_header* pMsg)
{
    IcdTxQueue->enqueue(*pMsg);
}


