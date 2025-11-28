#include "sEtherCAT_Interface.h"

//SLAVE_STRUCT epos3[3];
//#define MAXON_EPOS3  0x000000fb, 0x64400000
//#define EPOS3_0_Pos 0, 0
//#define EPOS3_1_Pos 0, 1
//#define EPOS3_2_Pos 0, 2
//#define NSEC_PER_SEC (1000000000L)

//#define FREQUENCY_MAIN_TASK         1000
//#define FREQUENCY_SUPERVISOR_TASK   100
//#define MAIN_TASK_PERIOD_NS         (NSEC_PER_SEC / FREQUENCY_MAIN_TASK)
//#define SUPERVISOR_TASK_PERIOD_NS   (NSEC_PER_SEC / FREQUENCY_SUPERVISOR_TASK)
//const static u_int16_t slave_pos[][2] = {{0, 0},
//                                         {0, 1},
//                                         {0, 2}};
sEtherCAT_Interface::sEtherCAT_Interface(QObject *parent) :
    QObject(parent),
    pEtherCAT_process(new QProcess(this))
{

}

sEtherCAT_Interface::~sEtherCAT_Interface(void)
{
}


void sEtherCAT_Interface::configure_slaves(void)
{
    int i;

    for (i = 0; i < NB_SLAVES; i++)
    {
        program = "/opt/etherlab/bin/ethercat -p";
        program.append(QString::number(i));
        program.append(QString(" --type uint8 download"));
        program.append(QString(" 0x1C12 0x00 0"));
        pEtherCAT_process->start(program);
        pEtherCAT_process->waitForFinished(1000000);

        program = "/opt/etherlab/bin/ethercat -p";
        program.append(QString::number(i));
        program.append(QString(" --type uint16 download"));
        program.append(QString(" 0x1C12 0x01 0x1603"));
        pEtherCAT_process->start(program);
        pEtherCAT_process->waitForFinished(1000000);

        program = "/opt/etherlab/bin/ethercat -p";
        program.append(QString::number(i));
        program.append(QString(" --type uint8 download"));
        program.append(QString(" 0x1C12 0x00 1"));
        pEtherCAT_process->start(program);
        pEtherCAT_process->waitForFinished(1000000);


        program = "/opt/etherlab/bin/ethercat -p";
        program.append(QString::number(i));
        program.append(QString(" --type uint8 download"));
        program.append(QString(" 0x1C13 0x00 0"));
        qDebug() << program;
        pEtherCAT_process->start(program);
        pEtherCAT_process->waitForFinished(1000000);

        program = "/opt/etherlab/bin/ethercat -p";
        program.append(QString::number(i));
        program.append(QString(" --type uint16 download"));
        program.append(QString(" 0x1C13 0x01 0x1A03"));
        pEtherCAT_process->start(program);
        pEtherCAT_process->waitForFinished(1000000);

        program = "/opt/etherlab/bin/ethercat -p";
        program.append(QString::number(i));
        program.append(QString(" --type uint8 download"));
        program.append(QString(" 0x1C13 0x00 1"));
        pEtherCAT_process->start(program);
        pEtherCAT_process->waitForFinished(1000000);
    }
    program = "sleep 5";
    pEtherCAT_process->start(program);
    pEtherCAT_process->waitForFinished(1000000);

    program = "/opt/etherlab/bin/ethercat rescan";
    pEtherCAT_process->start(program);
    pEtherCAT_process->waitForFinished(1000000);
    qDebug() << "Configuration Esclaves OK";
    emit ECSlave_configured();

}

void sEtherCAT_Interface::readout()
{
    qDebug() << pEtherCAT_process->readAllStandardOutput();
}
void sEtherCAT_Interface::readerr()
{
    qDebug() << pEtherCAT_process->readAllStandardError();
}
