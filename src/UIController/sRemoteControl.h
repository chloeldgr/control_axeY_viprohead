#ifndef SREMOTECONTROL_H
#define SREMOTECONTROL_H

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include "const_remote_control.h"

class sRemoteControl : public QObject
{
    Q_OBJECT
public:
    explicit sRemoteControl(QObject *parent = 0);
    ~sRemoteControl();
    void sendData(QStringList myFrame);
signals:
    void doCal(void);
    void startRTController(void);
    void stopRTController(void);
    void getRobotStatus(void);
public slots:
    // Communication TCP
    void acceptConnection();
    void startRead();
private :
    void        sendSynchroToClient(void);
    QTcpServer  *pServer;
    QTcpSocket  *pClient;
    int         requestID;
    int         number_of_parameters;
};

#endif // SREMOTECONTROL_H
