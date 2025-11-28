#include "sMasterRobot.h"
#include "../rtMasterController/rtMasterController.h"
#define RT_PROCESS 1
sMasterRobot::sMasterRobot(QObject *parent) :
    QObject(parent),
    pFIFO_cmd( new sRTFifo()),
    pFIFO_ackno( new sRTFifo()),
    pFIFO_status( new sRTFifo()),
    pFIFO_save_data( new sRTFifo()),
    pFIFO_param( new sRTFifo()),
    pFIFO_ident_param( new sRTFifo()),
    bFIFOIsOpen(false),
    controller_mode(WAIT_MODE),
    controller_submode(WAIT_SUBMODE),
    bIsCal(false),
    bIsRTRun(false),
    bIsPowerOn(false),
    pTimerRefresh(new QTimer),
    pTimerSaveData(new QTimer),
    p_rtController(new QProcess)
{
    pTimerRefresh->setInterval(100);
    pTimerSaveData->setInterval(10);

    connect(pTimerRefresh,  SIGNAL(timeout()),                  this,           SLOT(doRefreshStatus()));
    connect(pTimerSaveData, SIGNAL(timeout()),                  this,           SLOT(doSaveData()));
    connect(this,           SIGNAL(rtFIFO_opened()),            pTimerRefresh,  SLOT(start()));
    connect(this,           SIGNAL(rtFIFO_opened()),            pTimerSaveData, SLOT(start()));
    connect(this,           SIGNAL(rtFIFO_opened()),            this,           SLOT(doConnect()));
    connect(this,           SIGNAL(RefreshTimer_stopped()),     this,           SLOT(close_RTCommunication()));
    connect(this,           SIGNAL(rtController_started()),     this,           SLOT(open_RTCommunication()));
   // connect(p_rtController, SIGNAL(readyReadStandardError()),   this,           SLOT(updateError(p_rtController)));
   // connect(p_rtController, SIGNAL(readyReadStandardOutput()),  this,           SLOT(updateText(p_rtController)));
    // Création du fichier permettant la sauvegarde des données provenant de rtController
    // le fichier crée est temporaire
    data_file.setFileName("MasterData.dat");
    if (!data_file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;
    out_data.setDevice(&data_file);
    out_data << "% Fichier de sauvegarde : Projet S-Tronic Robot Maître\n"<< endl;
    out_data << "% (1)Temps (2)SlaveForce (3)SlavePosition1 (4)SlaveVel (5)SlaveControl (6)MasterForce (7)Masterposition (8)MasterVel (9)MasterControl" << endl;
    out_data.flush();
}

sMasterRobot::~sMasterRobot(void)
{

}
//void sMasterRobot::updateError(QProcess *myProcess)
//{
//    QByteArray data = p_rtController->readAllStandardError();
//    qDebug()<< QString(data);
//}

//void sMasterRobot::updateText(QProcess *myProcess)
//{
//    QByteArray data = p_rtController->readAllStandardOutput();
//    qDebug()<< QString(data);
//}

///
/// \fn     sRTInterface::open_RTCommunication()
/// \brief  Ouverture de la communication avec le controlleur temps-réel
///
void sMasterRobot::open_RTCommunication(void)
{
    try
    {
        qDebug() << "sMasterRobot : ouverture de la communication avec la couche RT";
        if ( !bIsRTRun )
            throw "rtController n'est pas en cours d'exécution - Impossible d'ouvrir les FIFOs";
        if ( bFIFOIsOpen)
            throw "MasterRobot : FIFO sont déjà ouvertes";
        // Ouverture de la FIFO permettant l'envoi de requÃªte de commande
        if ( pFIFO_cmd->Open("/dev/rtp21", O_RDWR|O_NONBLOCK) < 0)
            throw "Echec ouverture FIFO_CMD";
        if (pFIFO_ackno->Open("/dev/rtp22", O_RDWR|O_NONBLOCK) < 0)
            throw "Echec ouverture FIFO_ACKNO";
        if (pFIFO_status->Open("/dev/rtp23", O_RDONLY|O_NONBLOCK)< 0)
            throw "Echec ouverture FIFO_STATUS";
        if (pFIFO_save_data->Open("/dev/rtp24", O_RDWR|O_NONBLOCK)< 0)
            throw "Echec ouverture FIFO_SAVE_DATA";
        if (pFIFO_param->Open("/dev/rtp25", O_RDWR|O_NONBLOCK) < 0)
            throw "Echec ouverture FIFO_PARAM";
        if (pFIFO_ident_param->Open("/dev/rtp26", O_RDONLY|O_NONBLOCK)< 0)
            throw "Echec ouverture FIFO_IDENT_PARAM";


        qDebug() <<"sMasterRobot : Fifo ouvertes";

        bFIFOIsOpen = true;
        emit rtFIFO_opened();

    }
    catch(const char *e){
        qDebug() << "exception: " << e << endl;
    }

}


///
/// \fn     sRTInterface::close_RTCommunication
/// \brief  Fermeture de la communication avec le temps-réel
///
void sMasterRobot::close_RTCommunication(void)
{
    try
    {
        qDebug() <<"sMasterRobot : Fermeture des FIFOs.";
        if ( !bFIFOIsOpen)
            throw "MasterRobot : Les FIFO sont déjà fermées";
        pFIFO_cmd->Delete();
        pFIFO_ackno->Delete();
        pFIFO_status->Delete();
        pFIFO_save_data->Delete();
        pFIFO_ident_param->Delete();
        pFIFO_param->Delete();
        emit stopRTProcess();
        bFIFOIsOpen = false;
    }
    catch(int i)
    {
        qDebug() << "sMasterRobot : FIFO déjà fermées.";
    }
}

///
/// \fn     sMasterRobot::send_command
///  \brief Fonction permettant d'envoyer une commande vers le code bas niveau temps-réel
/// \param new_cmd
///
void sMasterRobot::send_command(cmd_type new_cmd)
{
    int ret;
    cmd_type recu;
    QTime dTime;
    try{

        /**
             * Ecriture de la nouvelle commande dans la fifo CMD
             **/


        if (  pFIFO_cmd->Write(&new_cmd, sizeof(cmd_type))< 0)
            throw  "sMasterRobot : Echec ecriture dans la fifo CMD ";

        /**
       * Attente accusÃ© de rÃ©ception
       **/
        dTime.restart();
        do{
            if ( dTime.elapsed()> 500)
                throw "sMasterRobot : TIMEOUT envoi commande";

        }while(pFIFO_ackno->Read(&recu, sizeof(cmd_type))<0);
        /**
       * Test AccusÃ© de rÃ©ception
       **/
        if (recu != new_cmd)
        {
            throw  "sMasterRobot : Echec accusé de réception";
        }
    }
    catch (const char* e) {
        qDebug() << e << endl;
    }
}

///
/// \fn sMasterRobot::startRTController
/// \brief Démarrage du code bas niveau temps-réel
///
void sMasterRobot::startRTController(void)
{
    if ( bIsRTRun )
        return;
    qDebug() << "sMasterRobot : Execution de rtMasterController";
#if RT_PROCESS == 1
    p_rtController->start("/root/develop/rtMasterController/rtMasterController");
    p_rtController->waitForStarted();
    sleep(2);
#endif
    bIsRTRun = true;
    bIsPowerOn = true;
    emit ControllerStatusChanged(bIsRTRun);
    emit rtController_started();

}

///
///
///  \fn sMasterRobot::stopRTController
/// \brief Arret du code bas niveau temps-réel
///
void sMasterRobot::stopRTController(void)
{

    qDebug() << "sMasterController : Appel de la fonction stopRTController";
    if ( !bIsRTRun)
        return;
    send_command(CMD_EXIT);
#if RT_PROCESS == 1
    p_rtController->kill();
#endif

    if ( pTimerRefresh->isActive())
        pTimerRefresh->stop();
    if ( pTimerSaveData->isActive())
        pTimerSaveData->stop();
    emit RefreshTimer_stopped();

    bIsRTRun    = false;
    bIsCal      = false; // A voir
    bIsPowerOn  = false;
    emit ControllerStatusChanged(bIsRTRun);
    emit CalStatusChanged(bIsCal);
    emit PowerStatusChanged(bIsPowerOn);

}



///
/// \fn     sRobotController::doCal
/// \brief  requête de calibration
///
void sMasterRobot::doCal(void)
{
    if ( !bIsCal)
        send_command(CMD_DO_CAL);

}
///
/// \brief sMasterRobot::doStartTeleop
///
void sMasterRobot::doStartTeleop(void)
{
    qDebug() <<" Bascule de robot maître en mode  téléopération";
    send_command(CMD_TELEOP_START);
}
///
/// \brief sMasterRobot::doStopTeleop
///
void sMasterRobot::doStopTeleop(void)
{
    send_command(CMD_TELEOP_STOP);
}


///
/// \fn         sRobotController::doSaveData
/// \brief      place les données dans un fichier
///
void sMasterRobot::doSaveData(void)
{
    int i,j, n_datas, n_read;

    if( !bFIFOIsOpen)
        return ;

    SAVE_DATA_STRUCT SaveData[MAX_BYTES_FIFO];
    n_read = 	pFIFO_save_data->Read(  SaveData,
                                        MAX_BYTES_FIFO*sizeof(SAVE_DATA_STRUCT));
    n_datas = n_read/(sizeof(SAVE_DATA_STRUCT));
    if (data_file.exists() )
    {

        for ( j = 0 ; j < n_datas ; j++ )
        {
            out_data << SaveData[j].current_time << " ";
            out_data << SaveData[j].slave_force << " "
                     << SaveData[j].slave_position << " "
                     << SaveData[j].slave_JVel << " "
                     << SaveData[j].slave_control << " "
                     << SaveData[j].master_force << " "
                     << SaveData[j].master_position << " "
                     << SaveData[j].master_JVel << " "
                     << SaveData[j].master_control;
            out_data << endl;
            out_data.flush();
        }

    }
}


void sMasterRobot::doRefreshStatus(void)
{
    int i;
    RTCONTROLLER_STRUCT local_controller;
    QStringList mode_list ;
    QString measures;

    mode_list <<"WAIT"
             << "CAL"
             <<"JVEL_CTRL"
            <<"JVEL_PLANNER"
           <<"JPOS_STATIC"
          <<"IDENT"
         <<"GOTO_INIT_POS"
        <<"POS_PLANNER"
       <<"SLAVE"
      <<"TELEOP"
     << "CONNECT HW"
     << "CONFIGURE HW"
     << "EXIT"
     << "DISCONNECT HW"
     << "CONTROL";
    //qDebug() << "sMasterRobot : Envoi de la commande GET STATUS";
    send_command(CMD_GET_STATUS);

    if ( pFIFO_status->Read(&local_controller,
                            sizeof(RTCONTROLLER_STRUCT))<0)
    {
        qDebug() << "sMasterRobot : erreur lecture fifo status";
        return;

    }
    bIsCal = local_controller.bIsCal;

    //    qDebug() << "sMasterRobot : "
    //             << " IsReady "<<local_controller.bIsReady
    //             << " IsConnected " << local_controller.bHWIsConnected
    //             << " IsConfigured " << local_controller.bHWIsConfigured
    //             << " EPOS3Status " << local_controller.EPOS3Status;
    measures = QString::number(local_controller.JPos);
    emit CalStatusChanged(bIsCal);
    emit PowerStatusChanged(bIsPowerOn);
    emit RTControllerModeChanged(mode_list.at(local_controller.mode));
    emit MeasuresChanged(measures);
    //emit CalStepChanged(local_controller.submode);

}

void sMasterRobot::doConnect(void)
{
    qDebug() << "sMasterRobot : Envoi de la commande CONNECT_HARDWARE "<< CMD_CONNECT_HARDWARE;
    send_command(CMD_CONNECT_HARDWARE);

}


void sMasterRobot::doDisconnect(void)
{
    qDebug() << "sMasterRobot: Envoi de la commande DISCONNECT_HARDWARE" ;
    send_command(CMD_DISCONNECT_HARDWARE);
}
