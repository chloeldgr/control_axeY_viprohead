#ifndef QTCONTROLLERSERVER_H
#define QTCONTROLLERSERVER_H

//#include <QTcpServer>
//#include <QTcpSocket>
//#include <QAbstractSocket>
//#include "qtcontrollerclient.h"
#include <QtCore>
#include <QtNetwork>


#define NB_WORDS 10 // nombre de mots contenu dans la trame
// Structure de la trame


//enum cmd_id {
//    START_CONTROLLER    = 00,
//    STOP_CONTROLLER     = 01,
//    GET_STATUS          = 02,
//    DO_CALIBRATION      = 03,
//    GOTO_JPOS           = 04,
//    GOTO_CPOS           = 05,
//    JVEL_CTRL           = 06,
//    GRASP_NEEDLE        = 07,
//    RELEASE_NEEDLE      = 08,
//    INSERT_NEEDLE       = 09,
//    START_TELEOPERATION = 10
//};

//class QtControllerServer : public QTcpServer
//{
//    Q_OBJECT
//public:
//    explicit QtControllerServer(QObject *parent = 0);
//    void StartServer();
//    QtControllerClient *client;

//signals:
//  //  void newclientconnected(void);
//    void receivedResult(int number);
//public slots:
//    void displayResult(int number);
//    void nouvelleConnection(void);

//};
class QtControllerServer : public QObject
{
    Q_OBJECT
public:
    explicit QtControllerServer(QObject *parent = 0);

signals:
    void dataReceived(QByteArray data);
public slots:
    void sendDataToClient(QByteArray data);

private slots :
    void newConnection();
    void disconnected();
    void readyRead();

private :
    QTcpServer *m_server;
    QTcpSocket  *m_socket;
    QHash<QTcpSocket*, QByteArray*> buffers;
    QHash<QTcpSocket*, qint32*> sizes;
protected :
    bool m_headerRead;
    unsigned int m_size_of_data_to_read;

};

#endif // QTCONTROLLERSERVER_H
