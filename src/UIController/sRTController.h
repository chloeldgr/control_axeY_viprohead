#ifndef SRTCONTROLLER_H
#define SRTCONTROLLER_H

#include <QObject>

#include <QProcess>
#include <QDebug>
#include <sRTFifo.h>
#include <sRobot.h>
#include <sRemoteControl.h>
#include <QTimer>
#include <QFile>
#include <QTime>
#include <QSocketNotifier>
#include <QAbstractSocket>
#include <sys/types.h>
#include <sys/socket.h>
#include "../rtController/rtController.h"
#include <bitset>

#define NB_FIFO 6

enum idFIFO {fifo_cmd, fifo_ackno, fifo_status, fifo_save_data, fifo_param, fifo_ident_param};

typedef struct{
    double AOUT[NB_ANALOG_OUTPUT];
    double DOUT[NB_DIG_OUTPUT];
}ADVANCED_DIRECT_CONTROL_STRUCT;



class sRTController : public QObject
{
    Q_OBJECT
public:
    explicit sRTController(QObject *parent = 0);
    ~sRTController();
    // Unix signal handler
    static void Usr1SignalHandler(int unused);

signals:
    void    isStarted(void);
    void    isStopped(void);
    void    rtFIFO_opened(void);
    void    rtFIFO_closed(void);
    void    ControllerStatusChanged(bool);
    void    RTControllerModeChanged(QString);
    void    MeasuresChanged(double*);
    void    CalStatusChanged(bool);
    void    PowerStatusChanged(bool);
    void    RefreshTimer_stopped(void);
    void    GraspingButtonActivated(bool);
public slots:
    void    printOutput(void);
    void    start(void);
    void    stop(void);
    void    openRTFIFO(void);
    void    closeRTFIFO(void);
    void    send_command(cmd_type new_cmd);
    //
    void    doRefreshStatus(void);
    void    doSaveData(void);
    void    doWriteDigOut(int);

    void    startTeleop(int,double,double,double,double);
    void    stopTeleop(void);
    void    doCtrlAuxeticInchworm(int,int,double,double,double,double);
    void    doCtrlFlatInchworm(int, double, bool);
    void    writeAnalogOutput(int *value);
    void    changeAnalogOutput(int value[6]);

    void    getRobotStatus(void);
    // Qt Signal Handler
    void handleSigUsr();


private :
    static int sigusrFd[2];
    double *measures;
    QSocketNotifier *snUsr;


protected :
    sRTFifo         *pFIFO[NB_FIFO];
    sRobot          *pRobot;
    sRemoteControl  *pRemoteControl;

    bool        bFIFOIsOpen;

    QProcess    *pProcess;
    QString     program;
    bool        bIsRunning;
    QByteArray  result;
    u_int       mode;
    u_int       submode;
    QStringList mode_list ;
    QTimer      *pTimerRefresh;
    QTimer      *pTimerSaveData;
    QFile           data_file;
    QTextStream     out_data;
};

#endif // SRTCONTROLLER_H
