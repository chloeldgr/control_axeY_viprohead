#include "qtcontrollerserver.h"
#include <QtEndian>

QtControllerServer::QtControllerServer(QObject *parent) : QObject(parent),
    m_server(new QTcpServer(this)),
    m_socket(new QTcpSocket)
{
    connect(m_server, SIGNAL(newConnection()), SLOT(newConnection()));
    qDebug() << "Listening:" << m_server->listen(QHostAddress::Any, 1024);

}

void QtControllerServer::newConnection()
{
    qDebug() << "Un nouveau client vient de se connecter";
    while(m_server->hasPendingConnections())
    {
//        QTcpSocket *m_socket = m_server->nextPendingConnection();
        m_socket = m_server->nextPendingConnection();
        connect(m_socket, SIGNAL(readyRead()), SLOT(readyRead()));
        connect(m_socket, SIGNAL(disconnected()), SLOT(disconnected()));
        QByteArray *buffer = new QByteArray();
        qint32 *s = new qint32(0);
        buffers .insert(m_socket, buffer);
        sizes.insert(m_socket, s);

    }

}


void QtControllerServer::disconnected()
{
    qDebug() << "Un client vient de se déconnecter";

    QByteArray *buffer = buffers.value(m_socket);
    qint32 *s = sizes.value(m_socket);
    m_socket->deleteLater();;
    delete buffer;
    delete s;
}

void QtControllerServer::readyRead()
{
    //QTcpSocket *m_socket = static_cast<QTcpSocket*>(sender());
    QByteArray *buffer = buffers.value(m_socket);
    qint32 *s = sizes.value(m_socket);
    qint32 size = *s;
        buffer->append(m_socket->readAll());
            emit dataReceived(*buffer);
            buffer->clear();

}


void QtControllerServer::sendDataToClient(QByteArray data)
{
    m_socket->write(data);
}

