#include "sEtherCAT.h"

sEtherCAT::sEtherCAT(QObject *parent) :
    QObject(parent),
    bIsOpen(false),
    bECMasterIsRunning(false),
    p_Process(new QProcess),
    pTimerRefresh(new QTimer)
{
    pTimerRefresh->setInterval(250);
    pTimerRefresh->start();

    connect(pTimerRefresh,  SIGNAL(timeout()),                  this, SLOT(refreshStatus()));
    connect(p_Process,      SIGNAL(readyReadStandardOutput()),  this, SLOT(printOutput()));
}

sEtherCAT::~sEtherCAT()
{
    pTimerRefresh->stop();

    delete p_Process;
    delete pTimerRefresh;
}


void sEtherCAT::printOutput(void)
{
    result = p_Process->readAllStandardOutput();
}


bool sEtherCAT::getECMasterStatus(void)
{
    return bECMasterIsRunning;
}

///
/// \brief sEtherCAT::openEtherCAT
/// \brief Ouverture de la communication EtherCAT
///
void sEtherCAT::open(void)
{
    try
    {
        if (bIsOpen)
            throw "EtherCAT Communication already open";
        QString program ="/etc/init.d/ethercat";
        QString action = "start";
        p_Process->start(program, QStringList() << action);
        if (!p_Process->waitForFinished())
            throw p_Process->errorString();
        bIsOpen = true;
    }
    catch(const char *e)
    {
        qDebug() << "sEtherCAT " << e;
    }

}

///
/// \brief sEtherCAT::closeEtherCAT
/// \brief Fermeture de la communication EtherCAT
///
void sEtherCAT::close(void)
{
    try
    {
        if (!bIsOpen)
            throw "EtherCAT Communication already close";

        QString program ="/etc/init.d/ethercat";
        QString action = "stop";
        p_Process->start(program, QStringList() << action);
        if (!p_Process->waitForFinished())
            throw p_Process->errorString();
        bIsOpen = false;
    }
    catch(const char *e)
    {
        qDebug() << "sEtherCAT " << e;
    }

}


///
/// \brief sEtherCAT::refreshStatus
/// \brief Etat courant de la communication sur le bus EtherCAT
///
void sEtherCAT::refreshStatus(void)
{
    QTextStream qout(stdout);

    try{
        QString program = "/etc/init.d/ethercat";
        QString action = "status";
        p_Process->setReadChannel(QProcess::StandardOutput);
        p_Process->setProcessChannelMode(QProcess::MergedChannels);

        p_Process->start(program, QStringList() << action);
        if (!p_Process->waitForStarted())
            throw p_Process->errorString();

        if ( result.contains("running"))
        {
            bECMasterIsRunning = true;
            emit ECMasterStatusChanged();
        }
        if (result.contains("dead"))
        {
            bECMasterIsRunning = false;
            emit ECMasterStatusChanged();
        }
    }
    catch(const char *e)
    {
        qDebug() << "sEtherCAT " << e;
    }

}
