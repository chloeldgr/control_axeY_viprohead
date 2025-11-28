#ifndef EXTERNALCONTROLSERVER_H
#define EXTERNALCONTROLSERVER_H

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QMessageBox>
#include <QAbstractSocket>
#include <QDebug>
#include <QThreadPool>
#include "qtcontrollerclient.h"


class externalControlServer : public QTcpServer
{
    Q_OBJECT
public:
    explicit externalControlServer(QObject *parent = 0);
    void envoyerATous(const QString &message);
    void startServer(void);

//signals:
//    void error(QTcpSocket::SocketError socketerror);
//    void serverStarted(void);
//    void serverStopped(void);
//    void connect_device(void);

//public slots:
//    void stopServer(void);

private :

   // QTcpServer *serveur;
    QList<QTcpSocket *>  clients;
    //QtControllerClient *pClient;
    //QList<QTcpSocket*> clients;
    qint16 tailleMessage;
private slots:
    void nouvelleConnexion(void);
    void donneesRecues(void);
    void deconnexionClient(void);
};
//private slots :
//    void demande_connexion() ;
//    void lecture();
//signals :
//    void vers_IHM_connexion();
//    void vers_IHM_texte(QString);
//private :
//    QTcpSocket *clientConnection;
//};

#endif // EXTERNALCONTROLSERVER_H
