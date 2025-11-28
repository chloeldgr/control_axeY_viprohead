#ifndef SMASTERROBOT_H
#define SMASTERROBOT_H

#include "../CommonLibs/rtControllerModules/rtController_Common_Parameters.h"
#include "sRTFifo.h"
#include <QObject>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QTime>
#include <QProcess>
#include <QThread>

class sMasterController : public QThread
{
    Q_OBJECT
    void run() Q_DECL_OVERRIDE{
        QString result;
        QProcess *myProcess = new QProcess(this);


           QString Path = "/root/develop/rtMasterController";


           myProcess->start(Path);
        emit resultReady(result);
    }
signals :
    void resultReady(const QString &s);
};

class sMasterRobot : public QObject
{
    Q_OBJECT
public:
    explicit sMasterRobot(QObject *parent = 0);
    ~sMasterRobot();
    void            doStartTeleop(void);
    void            doStopTeleop(void);
public slots:
    void            startRTController(void);
    void            stopRTController(void);
    void            open_RTCommunication(void);
    void            close_RTCommunication(void);
    void            doCal(void);
    void            doRefreshStatus(void);
    void            doSaveData(void);
    //void            ChangeSelectedJoint(int);
    void            doConnect(void);
    void            doDisconnect(void);
//    void            updateError(QProcess*);
//    void            updateText(QProcess*);
signals:
    void            ControllerStatusChanged(bool);
    void            RTControllerModeChanged(QString);
    void            MeasuresChanged(QString);
    void            CalStatusChanged(bool);
    void            CalStepChanged(int);
    void            SecurityStatusChanged(bool);
    void            PowerStatusChanged(bool);
    void            RefreshTimer_stopped(void);
    void            rtController_started(void);
    void            rtFIFO_opened(void);


    void            stopRTProcess();
protected :

    sRTFifo         *pFIFO_cmd;
    sRTFifo         *pFIFO_ackno;
    sRTFifo         *pFIFO_status;
    sRTFifo         *pFIFO_save_data;
    sRTFifo         *pFIFO_param;
    sRTFifo         *pFIFO_ident_param;


    bool            bFIFOIsOpen;
    unsigned int    controller_mode;
    unsigned int    controller_submode;


    QTimer          *pTimerRefresh;
    QTimer          *pTimerSaveData;

    bool            bIsCal;             /// flag indiquant que le robot est étalonné
    bool            bIsRTRun;           /// Flag indiquant que le controleur RT est en cours d'excécution
    bool            bIsPowerOn;         /// flag indiquant que le robot est sous tension

    QFile           data_file;
    QTextStream     out_data;
    // Fonctions membres protégées
    void send_command(cmd_type new_cmd);
    QProcess *p_rtController;
};

#endif // SMASTERROBOT_H

