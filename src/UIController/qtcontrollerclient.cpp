#include "qtcontrollerclient.h"

//QtControllerClient::QtControllerClient(QObject *parent) : QObject(parent)
//{
//    QThreadPool::globalInstance()->setMaxThreadCount(5);
//}

//QtControllerClient::~QtControllerClient()
//{

//}

//void QtControllerClient::SetSocket(int Descriptor)
//{
//    // make a new socket
//    socket = new QTcpSocket(this);


//    connect(socket, SIGNAL(connected()), this, SLOT(connected()));
//    connect(socket, SIGNAL(disconnected()), this, SLOT(disconnected()));
//    connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead()));

//    socket->setSocketDescriptor(Descriptor);

//    qDebug() << " Client connected at " << Descriptor;
//}
//// asynchronous - runs separately from the thread we created
//void QtControllerClient::connected()
//{
//    qDebug() << "Client connected event";
//}

//// asynchronous
//void QtControllerClient::disconnected()
//{
//    qDebug() << "Client disconnected";
//}

// Our main thread of execution
// This happening via main thread
// Our code running in our thread not in Qthread
// That's why we're getting the thread from the pool

void QtControllerClient::onNewData()
{
    QByteArray Buffer;
    qDebug() << "MyClient::readyRead()";
    Buffer =  m_socket->readAll();

    int commande = Buffer.toInt(0);
    qDebug()<< commande;
    // Time consumer
    QtControllerTask *mytask = new QtControllerTask(commande);
    mytask->setAutoDelete(true);
    connect(mytask, SIGNAL(Result(int)), this, SLOT(TaskResult(int)), Qt::QueuedConnection);

    qDebug() << "Starting a new task using a thread from the QThreadPool";
    QThreadPool::globalInstance()->start(mytask);

}

// After a task performed a time consuming task.
// We grab the result here, and send it to client
void QtControllerClient::TaskResult(int Number)
{
    QByteArray Buffer;

    Buffer.append("\r\nTask result = ");
    Buffer.append(QString::number(Number));

    m_socket->write(Buffer);

    emit receivedResult(Number);
}
