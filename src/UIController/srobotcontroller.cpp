#include "srobotcontroller.h"
#include "QtControllerConfig.h"

#include <QDebug>
#include <QMessageBox>

sRobotController::sRobotController(QObject *parent) :
    QObject(parent),
    id_fifo_cmd(0),
    id_fifo_ackno(0),
    id_fifo_status(0),
    id_fifo_save_data(0),
    bFIFOIsOpen(false),
    controller_mode(WAIT_MODE),
    controller_submode(WAIT_SUBMODE),
    pTimerRefresh(new QTimer),
    pTimerSaveData(new QTimer),
    process_rtController(new QProcess(this)),
    bIsCal(false),
    bIsRTRun(false),
    bIsPowerOn(false),
    bIsTeleopOn(false),
    JPos(NB_JOINTS),
    RefJPos(NB_JOINTS),
    RefCPos(NB_DOF),
    RefJVel(NB_JOINTS),
    JPosCal(NB_JOINTS),
    selected_joint(0),
    m_serveur(new QtControllerServer),
    pRobot(new sRobot)
{
    int i;
    pTimerRefresh->setInterval(100);
    pTimerSaveData->setInterval(10);
    connect(pTimerRefresh,  SIGNAL(timeout()),                      this,                       SLOT(doRefreshStatus()));
    connect(pTimerSaveData, SIGNAL(timeout()),                      this,                       SLOT(doSaveData()));
    connect(this,           SIGNAL(rtFIFO_opened()),                pTimerRefresh,              SLOT(start()));
    connect(this,           SIGNAL(rtFIFO_opened()),                pTimerSaveData,             SLOT(start()));
    connect(this,           SIGNAL(RefreshTimer_stopped()),         this,                       SLOT(close_RTCommunication()));
    connect(this,           SIGNAL(rtController_started()),         this,                       SLOT(open_RTCommunication()));
    connect(this,           SIGNAL(stopRTProcess()),                this->process_rtController, SLOT(terminate()));
    connect(m_serveur,      SIGNAL(dataReceived(QByteArray)),       this,                       SLOT(parserQtControllerServer(QByteArray)));
    connect(this,           SIGNAL(sendDataToServer(QByteArray)),   m_serveur,                  SLOT(sendDataToClient(QByteArray)));

    // Initialisation des paramètres du robot
    JPos.fill(0);
    RefJPos.fill(0);
    RefJVel.fill(0);
    RefCPos.fill(0);
    JPosCal.fill(0);
    // Création du fichier permettant la sauvegarde des données provenant de rtController
    open_save_file();
}

sRobotController::~sRobotController(void)
{

}


///
/// \fn sRobotController::open_save_file
/// \brief ouverture et formatage du fichier de sauvegarde complet
///
void sRobotController::open_save_file(void)
{
    // le fichier crée est temporaire
    data_file.setFileName("temp.dat");
    if (!data_file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;
    out_data.setDevice(&data_file);
    out_data << "% Fichier de sauvegarde : Projet S-Tronic \n"<< endl;
    out_data << "% (1)Temps (2)Force (3)JPos1 (4)JPos2 (5)JPos3 (6)Ref_JPos1 (7)Ref_JPos2 (8)Ref_JPos3 (9)U1 (10)U2 (11)U3" << endl;
    out_data.flush();
}

/// Accesseurs


/// Modifieurs
///

///
/// \brief sRobotController::setRefJPos
/// \param value
///
void sRobotController::setRefJPos(QVector<double> value)
{
    int i;
    try{
        if ( value.size() != NB_JOINTS)
            throw "Erreur : Nombre de paramètres erroné";
        for (i = 0; i < NB_JOINTS; i++)
            RefJPos[i] =  value[i];

    }
    catch(const char *e){
        qDebug() << "exception: " << e << endl;
    }

}

///
/// \brief sRobotController::setRefJPos
/// \param value
///
void sRobotController::setRefJVel(QVector<double> value)
{
    int i;
    try{
        if ( value.size() != NB_JOINTS)
            throw "Erreur : Nombre de paramètres erroné";
        for (i = 0; i < NB_JOINTS; i++)
            RefJVel[i] = value[i];

    }
    catch(const char *e){
        qDebug() << "exception: " << e << endl;
    }

}




///
/// \fn sRobotController::parserQtControllerServer
/// \brief Fonction interprétant la trame de données provenant d'un client
///        - Lecture de la trame :
///             - code correspondant à la requête
///             - le nombre de paramètres (si pas besoin mettre 0 0)
///             - si des paramètres sont passés en arguments il faut les séparer d'un espace
///                 les paramètres peuvent être des nombre réelles
///             - Après réception, le serveur envoi un accusé de réception
///                 - si tout est OK on renvoi le code de la requête
///                 - sinon on renvoi -1
///         - Envoi d'une trame :
///             - la réception d'une requête de lecture on renvoi une trame si tout est ok
///                 - code de la requête
///                 - nombre de paramètres
///                 - les paramètres séparés d'un espace
///             - Si pas OK on renvoi -1
/// \param data
///
void sRobotController::parserQtControllerServer(QByteArray data)
{
    QByteArray result;
    QByteArray arguments;
    QString buffer;
    QStringList str_arg;
    PARAM_CONTROLLER_STRUCT param;

    int i;
    // lecture du code de la requête
    QString str_cmd(data.mid(0,2));
    // lecture du nombre de paramètres
    QString str_nb_param(data.mid(2,4));
    // Interprétation requête et nombre paramètres
    quint16 nb_param = str_nb_param.toInt();
    quint16 cmd = str_cmd.toInt();    
    switch( cmd)
    {
    case SERVER_CMD_START_RT_CONTROLLER:
        // exécution de la commande startRTController
        startRTController();
        // Accusé de réception
        result.append(str_cmd);

        emit sendDataToServer(result);
        break;
    case SERVER_CMD_STOP_RT_CONTROLLER :
        //Appel de la fonction pour arrêter le controleur
        stopRTController();
        // Accusé de réception
        result.append(str_cmd);
        emit sendDataToServer(result);
        break;


    case SERVER_CMD_GET_STATUS:
        // Renvoi les paramètres courant de l'objet sRobotController
        // Mise en forme de la trame
        result.append(str_cmd);
        result.append(tr(" "));
        result.append(QString::number(4+NB_JOINTS));
        result.append(tr(" "));
        result.append(QString::number(bIsPowerOn));
        result.append(tr(" "));
        result.append(QString::number(bIsRTRun));
        result.append(tr(" "));
        result.append(QString::number(bIsCal));
        result.append(tr(" "));
        result.append(QString::number(bIsTeleopOn));

        for (i = 0; i < NB_JOINTS;i++)
        {
            result.append(tr(" "));
            result.append(QString::number(JPos.value(i)));
        }

        emit sendDataToServer(result);
        break;

    case SERVER_CMD_GOTO_CARTPOS:
        // réception de la commande pour un déplacement dans l'espace cartésien

        arguments.append(data.mid(4,data.size()));
        buffer = arguments;
        str_arg = buffer.split(' ', QString::SkipEmptyParts);
        // Si le nombre de paramètres correspond au nombre de degré de liberté
        if ( str_arg.size() == NB_DOF)
        {
            RefCPos.value(i,  str_arg.at(i).toDouble());
            result.append(str_cmd);
            gotoCPosPlanner();
        }
        else
            result.append(tr("-1"));

        emit sendDataToServer(result);
        break;

    case SERVER_CMD_GOTO_JOINTPOS:
        arguments.append(data.mid(4,data.size()));
        buffer = arguments;
        str_arg = buffer.split(' ', QString::SkipEmptyParts);

        // Si le nombre de paramètres correspond au nombre de degré de liberté
        if ( str_arg.size() == NB_JOINTS)
        {
            for (i = 0; i < NB_JOINTS;i++)
            {
                RefJPos.value(i,  str_arg.at(i).toDouble());
            }
            result.append(str_cmd);
            gotoJPosPlanner();

        }
        else
            result.append(tr("-1"));

        emit sendDataToServer(result);
        break;

    case SERVER_CMD_DO_CALIBRATION:
        // La procédure d'étalonnage nécessite des coordonnées articulaires
        arguments.append(data.mid(4,data.size()));
        buffer = arguments;
        str_arg = buffer.split(' ', QString::SkipEmptyParts);
        if ( str_arg.size() == NB_JOINTS)
        {
            for (i = 0; i < NB_JOINTS;i++)
                JPosCal.value(i,  str_arg.at(i).toDouble());
            result.append(str_cmd);
            doCal();
        }
        else
            result.append(tr("-1"));
        emit sendDataToServer(result);
        break;

    case SERVER_CMD_JVEL_CTRL:
        arguments.append(data.mid(4,data.size()));
        buffer = arguments;
        str_arg = buffer.split(' ', QString::SkipEmptyParts);

        // Si le nombre de paramètres correspond au nombre de degré de liberté
        if ( str_arg.size() == NB_JOINTS)
        {
            for (i = 0; i < NB_JOINTS;i++)
                RefJVel.value(i,  str_arg.at(i).toDouble());
            result.append(str_cmd);
            doJVelPlanner();
        }
        else
            result.append(tr("-1"));

        emit sendDataToServer(result);
        break;

    case SERVER_CMD_GRASP_NEEDLE:
        qDebug() << "sRobotController : réception de la commande GRASP_NEEDLE";
        doGraspNeedle();
        result.append(str_cmd);

        emit sendDataToServer(result);
        break;

    case SERVER_CMD_RELEASE_NEEDLE:
        qDebug() << "sRobotController : réception de la commande RELEASE_NEEDLE";
        doReleaseNeedle();
        result.append(str_cmd);

        emit sendDataToServer(result);
        break;

    case SERVER_CMD_INSERT_NEEDLE:
        qDebug() << "sRobotController : réception de la commande INSERT_NEEDLE";
        result.append(str_cmd);

        emit sendDataToServer(result);
        break;

    case SERVER_CMD_DO_SLAVE_MODE:
        qDebug() << "sRobotController : réception de la commande SLAVE  ";
        result.append(str_cmd);
        doSlaveMode();
        emit sendDataToServer(result);
        break;

    default:
        qDebug() << "sRobotController : réception de la commande autre";
        result.append(tr("-1"));

        emit sendDataToServer(result);
        break;
    }
}




///
/// \brief sRTInterface::readyReadStandardOutput
///
void sRobotController::readyReadStandardOutput()
{
    qDebug() << process_rtController->readAllStandardOutput();
}

///
/// \brief sRTInterface::readyReadStandardError
///
void sRobotController::readyReadStandardError()
{
    qDebug() << process_rtController->readAllStandardError();
}



void sRobotController::startRTController(void)
{

    QString program="/root/develop/rtController/rtController";
   //QString program="/root/develop/virtualRobot/virtualRobot";

    if ( bIsRTRun )
        return;
    qDebug()<< "Démarrage du controleur temps-réel";
        process_rtController->start(program);
        if( !process_rtController->waitForStarted(3000000))
        {
            QMessageBox::critical(0,
                                  tr("Fatal Error"),
                                  tr("Could not start the rtController."),
                                  tr("Quit"));
            exit(-1);
        }
    sleep(2);
    bIsRTRun = true;
    bIsPowerOn = true;

    emit ControllerStatusChanged(bIsRTRun);
    emit rtController_started();

}


void sRobotController::stopRTController(void)
{

    if ( !bIsRTRun)
        return;
    send_command(CMD_EXIT);

    if ( pTimerRefresh->isActive())
        pTimerRefresh->stop();
    if ( pTimerSaveData->isActive())
        pTimerSaveData->stop();
    emit RefreshTimer_stopped();

    bIsRTRun    = false;
    bIsCal      = false; // A voir
    bIsPowerOn  = false;
    bIsTeleopOn = false;
    emit ControllerStatusChanged(bIsRTRun);
    emit CalStatusChanged(bIsCal);
    emit PowerStatusChanged(bIsPowerOn);
}

///
/// \fn     sRTInterface::open_RTCommunication()
/// \brief  Ouverture de la communication avec le controlleur temps-réel
///
void sRobotController::open_RTCommunication(void)
{
    QString buf_string;
    // Si le controleur tems-réel n'est pas en cours d'excéution on ne peut pas ouvrir les FIFOs

    try
    {
        if ( !bIsRTRun )
            throw "rtController n'est pas en cours d'exécution - Impossible d'ouvrir les FIFOs";
        if ( bFIFOIsOpen)
            throw "FIFO sont déjà ouvertes";
        // Ouverture de la FIFO permettant l'envoi de requÃªte de commande
        id_fifo_cmd = open("/dev/rtp15", O_RDWR|O_NONBLOCK );
        if (id_fifo_cmd < 0)
            throw "echec ouverture FIFO_CMD";


        id_fifo_ackno = open("/dev/rtp16", O_RDWR|O_NONBLOCK );
        if ( id_fifo_ackno < 0 )
            throw "échec ouverture FIFO_ACKNO";

        id_fifo_status = open("/dev/rtp17", O_RDONLY|O_NONBLOCK );
        if ( id_fifo_status < 0 )
            throw "échec ouverture FIFO_STATUS";

        id_fifo_save_data = open("/dev/rtp18", O_RDONLY|O_NONBLOCK );
        if ( id_fifo_save_data < 0 )
            throw  "échec ouverture FIFO_SAVE_DATA";

        id_fifo_param = open("/dev/rtp19", O_RDWR|O_NONBLOCK );
        if ( id_fifo_param < 0 )
            throw "échec ouverture FIFO_PARAM";

        id_fifo_ident_param = open("/dev/rtp20", O_RDWR|O_NONBLOCK);
        if ( id_fifo_ident_param < 0)
            throw "échec ouverture FIFO_IDENT_PARAM";

        qDebug() <<"sRTInterface : Fifo ouvertes";
        emit rtFIFO_opened();

        bFIFOIsOpen = true;
    }
    catch(const char *e){
        qDebug() << "exception: " << e << endl;
    }

}


///
/// \fn     sRTInterface::close_RTCommunication
/// \brief  Fermeture de la communication avec le temps-réel
///
void sRobotController::close_RTCommunication(void)
{

    try
    {
        if ( !bFIFOIsOpen)
            throw FIFO_ALREADY_CLOSE;
        qDebug() <<"sRTInterface : Fermeture des FIFOs.";
        close(id_fifo_cmd);
        close(id_fifo_ackno);
        close(id_fifo_status);
        close(id_fifo_save_data);
        close(id_fifo_param);
        close(id_fifo_ident_param);
        //        emit stopRTProcess();
        process_rtController->close();
        bFIFOIsOpen = false;
    }
    catch(int i)
    {
        qDebug() << "sRTInterface : FIFO déjà fermées.";
    }
}

void sRobotController::send_command(cmd_type new_cmd)
{
    int ret;
    cmd_type recu;
    QTime dTime;
    try{

        /**
             * Ecriture de la nouvelle commande dans la fifo CMD
             **/
        ret =write (id_fifo_cmd, &new_cmd, sizeof(cmd_type));

        if (  ret< 0)
            throw  "sRTInterface : Echec ecriture dans la fifo CMD ";

        /**
       * Attente accusÃ© de rÃ©ception
       **/
        dTime.restart();
        do{
            if ( dTime.elapsed()> 500)
                throw "sRTInterface : TIMEOUT envoi commande";

        }while( (read(id_fifo_ackno, &recu, sizeof(cmd_type))< 0));
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
/// \fn     sRobotController::doCal
/// \brief  requête de calibration
///
void sRobotController::doCal(void)
{
    int ret, i;
    qDebug() << "sRobotController : envoi de la commande DO_CAL";
    PARAM_CONTROLLER_STRUCT local_param;
    // Modification des paramètres qui nous intéresse
    for (i = 0 ; i < NB_JOINTS ;i ++)
        local_param.cal_joint_position[i] = JPosCal[i];
    try{

        ret =write (id_fifo_param, &local_param, sizeof(PARAM_CONTROLLER_STRUCT));

        if (  ret< 0)
            throw  "sRTInterface : Echec ecriture dans la fifo CMD ";
        send_command(CMD_DO_CAL);
    }
    catch (const char* e) {
        qDebug() << "exception: " << e << endl;
    }
}


///
/// \fn         sRobotController::doSaveData
/// \brief      place les données dans un fichier
///
void sRobotController::doSaveData(void)
{
    int i,j, n_datas, n_read;

    if( !bFIFOIsOpen)
        return ;
    if ( !bIsCal)
        return ;
    SAVE_DATA_STRUCT SaveData[MAX_BYTES_FIFO];
    n_read = 	read(id_fifo_save_data,
                     SaveData,
                     MAX_BYTES_FIFO*sizeof(SAVE_DATA_STRUCT));
    n_datas = n_read/(sizeof(SAVE_DATA_STRUCT));
    if (data_file.exists() )
    {

        for ( j = 0 ; j < n_datas ; j++ )
        {
            out_data << SaveData[j].current_time << " ";
            for (i = 0 ; i < NB_FORCE_SENSORS;i++)
                out_data << SaveData[j].force[i] << " ";
            for (i = 0; i < NB_JOINTS; i++)
                out_data << SaveData[j].position[i] << " ";
            for (i = 0; i < NB_JOINTS; i++)
                out_data << SaveData[j].ref_jpos[i] << " ";
            for ( i = 0; i < NB_JOINTS; i++)
                out_data << SaveData[j].control[i] << " ";
            out_data << endl;
            out_data.flush();
        }

    }
}

///
/// \brief sRobotController::doRefreshStatus
///
void sRobotController::doRefreshStatus(void)
{
    CONTROLLER_STRUCT local_controller;
    send_command(CMD_GET_STATUS);
    if ( read(id_fifo_status,
              &local_controller,
              sizeof(CONTROLLER_STRUCT))<0)
    {
        qDebug() << "sRobotController : erreur lecture fifo status";
        return;

    }
    bIsCal = local_controller.bIsCal;
    for (int i = 0; i < NB_JOINTS;i++)
        JPos[i]= local_controller.JPos[i];
    emit CalStatusChanged(bIsCal);
    emit PowerStatusChanged(bIsPowerOn);
    emit RTControllerModeChanged(local_controller.mode);
    emit MeasuresChanged();
    emit CalStepChanged(local_controller.submode);
    emit sendChuckStatus(local_controller.bChuckIsClosed);

}


////
/// \fn    sRobotController::ChangeSelectedJoint
/// \brief permet de choisir un axe a controler
/// \param new_joint
///
void sRobotController::ChangeSelectedJoint(int new_joint)
{
    selected_joint = new_joint;

}

///
/// \brief sRobotController::doJVelControl
/// \param new_value
///
void sRobotController::doJVelPlanner(void)
{
    int ret, i ;
    // Envoi de la nouvelle valeur sur la fifo de param
    PARAM_CONTROLLER_STRUCT local_param;
    // Modification des paramètres qui nous intéresse
    for (i = 0 ; i < NB_JOINTS ;i ++)
        local_param.percent_maxJVel[i] = RefJVel[i]*0.01; // pourcentage de la vitesse max définie sur cette axe
//    qDebug() << RefJVel.at(0) << " " << RefJVel.at(1) << " "<< RefJVel.at(2) << " " << RefJVel.at(3);
    // écriture des nouveaux paramètres dans la structure et envoi dans la fifo
    try{

        ret =write (id_fifo_param, &local_param, sizeof(PARAM_CONTROLLER_STRUCT));

        if (  ret< 0)
            throw  "sRTInterface : Echec ecriture dans la fifo CMD ";
        // Envoi de la commande CMD_JVEL
        send_command(CMD_JVEL_PLANNER);
    }
    catch(const char* e) {
        qDebug() << "exception: " << e << endl;
    }

}


///
/// \brief sRobotController::doStaticJPos
///
void sRobotController::doJPosStatic(void)
{
    send_command(CMD_JPOS_STATIC);

}



void sRobotController::doIdentification(const IDENT_PARAM_STRUCT &param)
{
    int ret;

    try{
        ret =write (id_fifo_ident_param, &param, sizeof(IDENT_PARAM_STRUCT));

        if (  ret< 0)
            throw  "sRTInterface : Echec ecriture dans la fifo param ident ";
        send_command(CMD_DO_IDENT);
    }
    catch(const char* e) {
        qDebug() << "exception: " << e << endl;
    }

}



void sRobotController::gotoJPosPlanner(void)
{
    int ret, i;
    PARAM_CONTROLLER_STRUCT param;
    try{
        qDebug() << "Réception de la commande GOTO JPOS";
        for (i = 0 ; i < NB_JOINTS-1 ;i++)
            param.ref_joint_position[i] = RefJPos.at(i);

        ret = write(id_fifo_param, &param, sizeof(PARAM_CONTROLLER_STRUCT));
        if ( ret < 0)
            throw "sRobotController:  échec écriture dans la fifo param";
        send_command(CMD_GOTO_JPOS);

    }
    catch(const char *e){
        qDebug()<< "exception : "<<e;
    }
}

///
/// \fn sRobotController::gotoCPosPlanner
/// \brief  Déplacement du système robotique vers une consigne opérationnelle
///
void sRobotController::gotoCPosPlanner(void)
{
    int i,ret;
    PARAM_CONTROLLER_STRUCT param;

    try{
        // Réception de la consigne en positon cartésienne zf
        // Utilisation du modèle géométrique inverse pour calculer les consignes articulaires
        RefJPos = pRobot->ComputeIK(RefCPos);
        for (i = 0 ; i < NB_JOINTS-1 ;i++)
            param.ref_joint_position[i] = RefJPos.at(i);

        ret = write(id_fifo_param, &param, sizeof(PARAM_CONTROLLER_STRUCT));
        if ( ret < 0)
            throw "sRobotController:  échec écriture dans la fifo param";
        send_command(CMD_GOTO_JPOS);

    }
    catch(const char *e)
    {
        qDebug() << "Gestion Exception : " << e;
    }
}


///
/// \fn sRobotController::doGraspNeedle
/// \brief envoi une requête au controleur pour saisir l'aiguille
/// \arg aucun
/// \return aucun
///
void sRobotController::doGraspNeedle(void)
{
        send_command(CMD_CLOSE_CHUCK);
}

///
/// \fn     sRobotController::doReleaseNeedle
/// \brief  envoi un requête au controleur pour relacher l'aiguille
/// \arg    aucun
/// \aucun  aucun
///
void sRobotController::doReleaseNeedle(void)
{
        send_command(CMD_OPEN_CHUCK);
}

///
/// \fn     sRobotController::doSlaveMode
/// \brief  Envoi la requête au controleur pour basculer le robot en mode esclave
/// \arg    aucun
/// \return aucun
///
void sRobotController::doSlaveMode(void)
{
    send_command(CMD_SLAVE);
}
