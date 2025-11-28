#include "sRemoteControl.h"



sRemoteControl::sRemoteControl(QObject *parent) : QObject(parent),
    pServer(new QTcpServer),
    pClient(new QTcpSocket)
{
    connect(pServer,        SIGNAL(newConnection()),            this,   SLOT(acceptConnection()));
    pServer->listen(QHostAddress::Any, 8888);

}

sRemoteControl::~sRemoteControl()
{
    if (pServer->isListening())
        pServer->close();
    if ( pClient->isOpen())
        pClient->close();
}

void sRemoteControl::acceptConnection()
{
    pClient = pServer->nextPendingConnection();
    connect(pClient, SIGNAL(readyRead()), this, SLOT(startRead()));
    // On confirme au client qui vient de se connecter que la connection est acceptée
    // On envoi un signal de synchro
    sendSynchroToClient();
}

void sRemoteControl::sendData(QStringList myFrame)
{
    QString buf;
    buf = myFrame.join("\n");
    pClient->write(buf.toStdString().c_str());
}

void sRemoteControl::sendSynchroToClient(void)
{
  //pClient->write(QString(SYNCHRO_COM_TCP).toStdString().c_str());
}

void sRemoteControl::startRead()
{
    char buffer[1024] ={0};
    pClient->read(buffer, pClient->bytesAvailable());
    // Interpréteur  de trame
    QString myFrame = QString(buffer);
    QStringList frameWord = myFrame.split(" ");
    requestID               = frameWord.at(0).toInt();
    number_of_parameters    = frameWord.at(1).toInt();
    switch(requestID)
    {
    // Réception de la requête pour lancer le controleur temps-réel
    case REMOTE_CTRL_START_RTCONTROLLER :

        emit startRTController();
        sendSynchroToClient();
        break;
        // Réception de la requête pour stopper le controleur temps-réel
    case REMOTE_CTRL_STOP_RTCONTROLLER :
        emit stopRTController();
        sendSynchroToClient();
        break;
        //Réception de la requête pour récupérer les coordonnées articulaires
    case REMOTE_CTRL_GET_ROBOT_STATUS:
        // il y a normalement un paramètre
        emit getRobotStatus();
        break;
    case REMOTE_CTRL_SET_JPOS_REF:

        break;
    default :
        break;
    }

 //   pClient->close();
}
