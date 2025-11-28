#include "sRTController.h"


int sRTController::sigusrFd[2];

sRTController::sRTController(QObject *parent) : QObject(parent),
    measures(new double[NB_ANALOG_INPUT]),
    bFIFOIsOpen(false),
    pRobot(new sRobot),
    pRemoteControl(new sRemoteControl),
    pProcess(new QProcess),
    program("./rtController/rtController"),
    bIsRunning(false),
    mode(WAIT_MODE),
    submode(WAIT_SUBMODE),
    pTimerRefresh(new QTimer),
    pTimerSaveData(new QTimer)
{
    int i;
    for (i = 0 ; i < NB_FIFO;i++)
        pFIFO[i] = new sRTFifo;

    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sigusrFd))
        qFatal("Impossible de créer USR Socketpair");
    snUsr = new QSocketNotifier(sigusrFd[1], QSocketNotifier::Read, this);


    pTimerRefresh->setInterval(100);
    pTimerSaveData->setInterval(10);

    connect(snUsr,          SIGNAL(activated(int)),             this,           SLOT(handleSigUsr()));
    connect(pProcess,       SIGNAL(readyReadStandardOutput()),  this,           SLOT(printOutput()));
    connect(this,           SIGNAL(rtFIFO_opened()),            pTimerRefresh,  SLOT(start()));
    connect(this,           SIGNAL(rtFIFO_opened()),            pTimerSaveData, SLOT(start()));
    connect(this,           SIGNAL(isStarted()),                this,           SLOT(openRTFIFO()));
    connect(pTimerRefresh,  SIGNAL(timeout()),                  this,           SLOT(doRefreshStatus()));
    connect(pTimerSaveData, SIGNAL(timeout()),                  this,           SLOT(doSaveData()));

    // Connection entre les signaux reçues par RemoteControl et les actions RTController
    connect(pRemoteControl, SIGNAL(startRTController()),        this,           SLOT(start()));
    connect(pRemoteControl, SIGNAL(stopRTController()),         this,           SLOT(stop()));
    connect(pRemoteControl, SIGNAL(getRobotStatus()),           this,           SLOT(getRobotStatus()));



    // Création du fichier permettant la sauvegarde des données provenant de rtController
    // le fichier crée est temporaire
    data_file.setFileName("temp.dat");
    if (!data_file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;
    out_data.setDevice(&data_file);
    out_data << "% Fichier de sauvegarde : Inchworm \n"<< endl;
    out_data << "% ED1-->Pression1 (Auxetique) ED2-->Pression2 (Gripper1) ED3-->Pression3 (Gr2) EV1 EV2 EV3 EV4"<<endl;
    out_data << "% (1)Temps (2)Pression1 (3)Pression2 (4)Pression3 (5) Pression4 (6)Input 1 EL3104_1 (7)Input2 EL3104_1 (8)Input3 EL3104_1 (9)Input4 EL3104_1 (10)EV1 (11)EV2 (12)EV3 (13)EV4" << endl;

    out_data.flush();
    mode_list<<"WAIT"<< "CAL"<<"JVEL_CTRL"<<"JVEL_PLANNER"<<"JPOS_STATIC"<<"IDENT"<<"GOTO_INIT_POS"<<"POS_PLANNER"<<"CTRL INCHWORM"<<"CTRL FLAT INCHWORM"<<"TELEOP";
}

sRTController::~sRTController()
{
    delete pProcess;
    delete snUsr;
}




void sRTController::printOutput(void)
{
    result = pProcess->readAllStandardOutput();
}

///
/// \brief sRTController::getRobotStatus
///\brief callback associé au signal getRobotStatus de remoteControl
void sRTController::getRobotStatus(void)
{
    int i;
    QStringList current_frame;
    //    for ( i = 0 ; i < NB_JOINTS;i++)
    //        current_frame << QString::number(pRobot->getJPos(i));
    //    current_frame << QString::number(pRobot->getForces()[0]);
    //    current_frame << QString::number(pRobot->getCalStatus());
    //    current_frame << QString::number(pRobot->getPowerStatus());
    pRemoteControl->sendData(current_frame);
}



void sRTController::Usr1SignalHandler(int unused)
{
    char a = 1;
    ::write(sigusrFd[0], &a, sizeof(a));
}
void sRTController::handleSigUsr()
{
    snUsr->setEnabled(false);
    char tmp;
    ::read(sigusrFd[1], &tmp, sizeof(tmp));
    //
    qDebug() << "Catch sigusr";
    bIsRunning = true;
    emit ControllerStatusChanged(bIsRunning);
    emit isStarted();

    snUsr->setEnabled(true);

}

///
/// \brief sRTController::start
///
void sRTController::start(void)
{
    try
    {
        if (bIsRunning)
            throw "RTController already started";
        pProcess->start(program);

        if (!pProcess->waitForStarted())
            throw pProcess->errorString();

    }
    catch(const char *e)
    {
        qDebug() << "sRTController " << e;
    }
}

///
/// \brief sRTController::stop
///
void sRTController::stop(void)
{
    try
    {
        if (!bIsRunning)
            throw "RTController already stopped";
        pProcess->kill();
        pTimerRefresh->stop();
        pTimerSaveData->stop();
        bIsRunning = false;
        closeRTFIFO();
        emit ControllerStatusChanged(bIsRunning);
        emit PowerStatusChanged(pRobot->getPowerStatus());
        emit CalStatusChanged(pRobot->getCalStatus());

    }
    catch(const char *e)
    {
        qDebug() << "sRTController " << e;
    }
}

///
/// \brief sRTController::openRTFIFO
///
void sRTController::openRTFIFO(void)
{
    // Si le controleur tems-réel n'est pas en cours d'excéution on ne peut pas ouvrir les FIFOs
    try
    {
        qDebug() << "Ouverture des FIFOs";
        //        if ( !bIsRunning )
        //            throw "rtController n'est pas en cours d'exécution - Impossible d'ouvrir les FIFOs";
        if ( bFIFOIsOpen)
            throw "FIFO déjà ouvertes";
        // Ouverture de la FIFO permettant l'envoi de requÃªte de commande
        if ( pFIFO[fifo_cmd]->Open("/dev/rtp15", O_RDWR|O_NONBLOCK) < 0)
            throw "Echec ouverture FIFO_CMD";
        if (pFIFO[fifo_ackno]->Open("/dev/rtp16", O_RDWR|O_NONBLOCK) < 0)
            throw "Echec ouverture FIFO_ACKNO";
        if (pFIFO[fifo_status]->Open("/dev/rtp17", O_RDONLY|O_NONBLOCK)< 0)
            throw "Echec ouverture FIFO_STATUS";
        if (pFIFO[fifo_save_data]->Open("/dev/rtp18", O_RDWR|O_NONBLOCK)< 0)
            throw "Echec ouverture FIFO_SAVE_DATA";
        if (pFIFO[fifo_param]->Open("/dev/rtp19", O_RDWR|O_NONBLOCK) < 0)
            throw "Echec ouverture FIFO_PARAM";
        bFIFOIsOpen = true;
        emit rtFIFO_opened();
    }
    catch(const char *e){
        qDebug() << "sRTController (Open RTFIFO): " << e << endl;
    }

    emit rtFIFO_opened();
}

///
/// \brief sRTController::closeRTFIFO
///
void sRTController::closeRTFIFO(void)
{
    try
    {
        if ( !bFIFOIsOpen)
            throw "rtFIFO déjà fermées";

        pFIFO[fifo_cmd]->Delete();
        pFIFO[fifo_ackno]->Delete();
        pFIFO[fifo_status]->Delete();
        pFIFO[fifo_save_data]->Delete();
        pFIFO[fifo_ident_param]->Delete();

        bFIFOIsOpen = false;
        emit rtFIFO_closed();
    }
    catch(const char *e)
    {
        qDebug() << "sRTController: "<< e << endl;
    }
}


///
/// \brief sSlaveRobot::send_command
/// \param new_cmd
///
void sRTController::send_command(cmd_type new_cmd)
{
    int ret;
    cmd_type recu;
    QTime dTime;
    try{

        /**
             * Ecriture de la nouvelle commande dans la fifo CMD
             **/


        if (  pFIFO[fifo_cmd]->Write(&new_cmd, sizeof(cmd_type))< 0)
            throw  "sRTInterface : Echec ecriture dans la fifo CMD ";

        /**
       * Attente accusÃ© de rÃ©ception
       **/
        dTime.restart();
        do{
            if ( dTime.elapsed()> 500)
                throw "sRTInterface : TIMEOUT envoi commande";

        }while(pFIFO[fifo_ackno]->Read(&recu, sizeof(cmd_type))<0);
        /**
       * Test AccusÃ© de rÃ©ception
       **/
        if (recu != new_cmd)
        {
            throw  "sRTInterface : Echec accusé de réception";
        }
    }
    catch (const char* e) {
        qDebug() << "exception: " << e << endl;
    }
}




///
/// \brief sRobot::doRefreshStatus
/// \brief Handler
///
void sRTController::doRefreshStatus(void)
{
    int i;
    RTCONTROLLER_STRUCT local_controller;


    // Envoi de la requête afin de récupérer l'état courant du controleur bas niveau et informations sur le matériel
    send_command(CMD_GET_STATUS);

    // Lecture de la fifo contenant toutes les informations nécessaires à l'état courant
    if (  pFIFO[fifo_status]->Read( &local_controller,
                                    sizeof(RTCONTROLLER_STRUCT))<0)
    {
        qDebug() << "sRobotController : erreur lecture fifo status";
        return;
    }
    // Récupération des tensions analogiques
    for (i = 0 ; i < NB_ANALOG_INPUT;i++)
    {
        measures[i] = local_controller.analog_input[i];
    //    qDebug() << i << " " <<measures[i] ;
    }
    //qDebug() << "";
    emit GraspingButtonActivated(local_controller.digital_input[1]&(1u <<5));
    emit MeasuresChanged(measures);

}



///
/// \fn         sRobotController::doSaveData
/// \brief      place les données dans un fichier
///
void sRTController::doSaveData(void)
{
    int i,j, n_datas, n_read;

    SAVE_DATA_STRUCT SaveData[MAX_BYTES_FIFO];
    n_read = 	pFIFO[fifo_save_data]->Read(    SaveData,
                                                MAX_BYTES_FIFO*sizeof(SAVE_DATA_STRUCT));

    n_datas = n_read/(sizeof(SAVE_DATA_STRUCT));
    if (data_file.exists() )
    {

        for ( j = 0 ; j < n_datas ; j++ )   // Echantillonnage 1 kHz
        //for ( j = n_datas-1 ; j < n_datas ; j++ )    // Echantillonnage 10 fois plus petit
        {
            out_data    << SaveData[j].current_time << " "
                        << SaveData[j].analog_input[0] << " "
                        << SaveData[j].analog_input[1] << " "
                        << SaveData[j].analog_input[2] << " "
                        << SaveData[j].analog_input[3] << " "
                        << SaveData[j].analog_input[4] << " "
                        << SaveData[j].analog_input[5] << " "
                        << SaveData[j].analog_input[6] << " "
                        << SaveData[j].analog_input[7] << " ";
            if ( SaveData[j].digital_output & (1u << 0))
                out_data << " 1 ";
            else
                out_data << " 0 ";
            if ( SaveData[j].digital_output & (1u << 1))
                out_data << " 1 ";
            else
                out_data << " 0 ";
            if ( SaveData[j].digital_output & (1u << 2))
                out_data << " 1 ";
            else
                out_data << " 0 ";
            if ( SaveData[j].digital_output & (1u << 3))
                out_data << " 1 ";
            else
                out_data << " 0 ";

            out_data << endl;
            out_data.flush();
        }

    }
}


///
/// \fn sRTController::doWriteDigOut
/// \brief envoi d'un registre de 16bits pour contrôler les sorties numériques du banc pneumatique
/// \param reg
///
void sRTController::doWriteDigOut(int reg)
{
    int i,ret;
    RTCONTROLLER_PARAM_STRUCT local_controller_parameters;
    local_controller_parameters.reg_dig_output = reg;

    // Envoi des paramètres dans la FIFO param
    try{
        ret = pFIFO[fifo_param]->Write(&local_controller_parameters,sizeof(RTCONTROLLER_PARAM_STRUCT));
        if (  ret< 0)
            throw  "sRTInterface : Echec ecriture dans la fifo param ";
        // Envoi de la commande WRITE_DIG_OUT
        send_command(CMD_WRITE_DIG_OUT);
    }
    catch(const char* e) {
        qDebug() << "exception: " << e << endl;
    }

}

///
/// \fn sRTController::doWriteDigOut5V
/// \brief envoi d'un registre de 16bits pour contrôler les sorties numériques du banc pneumatique
/// \param reg
///
//void sRTController::doWriteDigOut5V(int reg)
//{
//    int i,ret;
//    RTCONTROLLER_PARAM_STRUCT local_controller_parameters;
//    local_controller_parameters.reg_dig_output_5V = reg;

//    // Envoi des paramètres dans la FIFO param
//    try{
//        ret = pFIFO[fifo_param]->Write(&local_controller_parameters,sizeof(RTCONTROLLER_PARAM_STRUCT));
//        if (  ret< 0)
//            throw  "sRTInterface : Echec ecriture dans la fifo param ";
//        // Envoi de la commande WRITE_DIG_OUT
//        send_command(CMD_WRITE_DIG_OUT_5V);
//    }
//    catch(const char* e) {
//        qDebug() << "exception: " << e << endl;
//    }

//}
void sRTController::changeAnalogOutput(int value[6])
{
    int ret,i;
    RTCONTROLLER_PARAM_STRUCT local_controller_parameters;
    for (i=0; i < 6 ;i++)
    {
        local_controller_parameters.analog_output[i] = (double)value[i]/1000;
       //qDebug() << " P"<<i<<" = " << local_controller_parameters.analog_output[i];
    }

    // Envoi des paramètres dans la FIFO param
    try{
        ret = pFIFO[fifo_param]->Write(&local_controller_parameters,sizeof(RTCONTROLLER_PARAM_STRUCT));
        if (  ret< 0)
            throw  "sRTInterface : Echec ecriture dans la fifo param ";
        // Envoi de la commande WRITE_ANA_OUT
        send_command(CMD_WRITE_ANA_OUT);
    }
    catch(const char* e) {
        qDebug() << "exception: " << e << endl;
    }

}


void sRTController::doCtrlAuxeticInchworm(int new_cmd,
                                          int direction,
                                          double V_VPPM1,
                                          double V_VPPM2,
                                          double ms_SeqPeriod,
                                          double ms_GripperPeriod)
{
    int ret;
    RTCONTROLLER_PARAM_STRUCT local_param;
    // Récupération des informations de la fenêtre de paramétrage de l'actionnement inchworm
    local_param.analog_output[ANA_OUT_1]    = V_VPPM1;

    local_param.analog_output[ANA_OUT_2]    = V_VPPM2;
    local_param.Inchworm_period_ms          = ms_SeqPeriod;
    local_param.Inchworm_chuck_time_ms      = ms_GripperPeriod;
    local_param.Inchworm_Direction          = (int)direction;
    qDebug() << "Ctrl Auxetic Inchworm : "  << local_param.analog_output[ANA_OUT_1] <<  "\n"
                                            << local_param.analog_output[ANA_OUT_2] <<  "\n"
                                            << local_param.Inchworm_period_ms <<  "\n"
                                            << local_param.Inchworm_chuck_time_ms <<  "\n"
                                            << local_param.Inchworm_Direction ;
    // Envoi des paramètres dans la FIFO param
    try{
        switch(new_cmd)
        {
        case 501:
            ret = pFIFO[fifo_param]->Write(&local_param,sizeof(RTCONTROLLER_PARAM_STRUCT));
            if (  ret< 0)
                throw  "sRTInterface : Echec ecriture dans la fifo param ";
            // Envoi de la commande WRITE_ANA_OUT
            send_command(CMD_INIT_INCHWORM);
            break;
        case 502:
            ret = pFIFO[fifo_param]->Write(&local_param,sizeof(RTCONTROLLER_PARAM_STRUCT));
            if (  ret< 0)
                throw  "sRTInterface : Echec ecriture dans la fifo param ";
            // Envoi de la commande WRITE_ANA_OUT
            send_command(CMD_CTRL_INCHWORM);
            break;
        case 503 :
            qDebug() << "Envoi de la commande STOP INCHWORM" ;
            send_command(CMD_STOP_INCHWORM);
            break;
        default:
            break;

        }

    }
    catch(const char* e) {
        qDebug() << "exception: " << e << endl;
    }
}



void sRTController::doCtrlFlatInchworm(int value1, double value2, bool value3)
{
    int ret;
    RTCONTROLLER_PARAM_STRUCT local_param;
    // Récupération des informations de la fenêtre de paramétrage de l'actionnement inchworm
    local_param.Inchworm_DC                 = value1;
    local_param.Inchworm_period_ms          = 1/value2*1000;
    local_param.Inchworm_Direction          = value3;
    qDebug() << local_param.Inchworm_DC << " "<< local_param.Inchworm_period_ms << " "<< local_param.Inchworm_Direction;

    // Envoi des paramètres dans la FIFO param
    try{
        ret = pFIFO[fifo_param]->Write(&local_param,sizeof(RTCONTROLLER_PARAM_STRUCT));
        if (  ret< 0)
            throw  "sRTInterface : Echec ecriture dans la fifo param ";
        // Envoi de la commande WRITE_ANA_OUT
        send_command(CMD_CTRL_FLAT_INCHWORM);
    }
    catch(const char* e) {
        qDebug() << "exception: " << e << endl;
    }
}


void sRTController::writeAnalogOutput(int *value)
{
    int ret;
    RTCONTROLLER_PARAM_STRUCT local_controller_parameters;
    local_controller_parameters.analog_output[ANA_OUT_5] = (double)value[0]/1000;
    local_controller_parameters.analog_output[ANA_OUT_6] = (double)value[1]/1000;
    // Envoi des paramètres dans la FIFO param
    try{
        ret = pFIFO[fifo_param]->Write(&local_controller_parameters,sizeof(RTCONTROLLER_PARAM_STRUCT));
        if (  ret< 0)
            throw  "sRTInterface : Echec ecriture dans la fifo param ";
        // Envoi de la commande WRITE_ANA_OUT
        send_command(CMD_WRITE_ANA_OUT);
    }
    catch(const char* e) {
        qDebug() << "exception: " << e << endl;
    }
}

void sRTController::startTeleop( int direction,
                                 double V_VPPM1,
                                 double V_VPPM2,
                                 double ms_SeqPeriod,
                                 double ms_GripperPeriod)
{
    int ret;
    qDebug() << "Start teleop" ;
    RTCONTROLLER_PARAM_STRUCT local_param;
    // Récupération des informations de la fenêtre de paramétrage de l'actionnement inchworm
    local_param.analog_output[ANA_OUT_1]    = V_VPPM1;

    local_param.analog_output[ANA_OUT_2]    = V_VPPM2;
    local_param.Inchworm_period_ms          = ms_SeqPeriod;
    local_param.Inchworm_chuck_time_ms      = ms_GripperPeriod;
    local_param.Inchworm_Direction          = (int)direction;
    qDebug() << "Ctrl Auxetic Inchworm : "  << local_param.analog_output[ANA_OUT_1] <<  "\n"
                                            << local_param.analog_output[ANA_OUT_2] <<  "\n"
                                            << local_param.Inchworm_period_ms <<  "\n"
                                            << local_param.Inchworm_chuck_time_ms <<  "\n"
                                            << local_param.Inchworm_Direction ;
    ret = pFIFO[fifo_param]->Write(&local_param,sizeof(RTCONTROLLER_PARAM_STRUCT));
    if (  ret< 0)
        throw  "sRTInterface : Echec ecriture dans la fifo param ";
    send_command(CMD_START_TELEOP);
}


void sRTController::stopTeleop(void)
{
    qDebug() <<"Stop teleop";
    send_command(CMD_STOP_TELEOP);
}
