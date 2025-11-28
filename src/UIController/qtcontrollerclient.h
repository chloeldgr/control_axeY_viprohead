#ifndef QTCONTROLLERCLIENT_H
#define QTCONTROLLERCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QDebug>
#include <QThreadPool>
#include "qtcontrollertask.h"
#include <QMessageBox>
#include <iostream>
class QtControllerClient : public QObject
{
    Q_OBJECT
public:
    QtControllerClient(QTcpSocket *socket) : m_socket(socket)
    {
        connect(socket, SIGNAL(readyRead()), this, SLOT(onNewData()));
    }

signals:
    void receivedResult(int number);
private slots:
    void onNewData();

public slots:
//    void connected();
//    void disconnected();
//    void readyRead();

    // make the server fully ascynchronous
    // by doing time consuming task
    void TaskResult(int Number);

private:
    QTcpSocket *m_socket;

};

#endif // QTCONTROLLERCLIENT_H
