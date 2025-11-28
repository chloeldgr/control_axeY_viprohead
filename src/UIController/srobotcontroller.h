#ifndef SROBOTCONTROLLER_H
#define SROBOTCONTROLLER_H

#include <QObject>
#include <QTimer>
#include <QTime>
#include <QProcess>
#include <QFile>
#include <QVector>
#include <QTextStream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <QTcpServer>
#include <QTcpSocket>
#include "../rtController/rtController.h"
#include <sRobot.h>
#include <qtcontrollerserver.h>


#define SERVER_CMD_START_RT_CONTROLLER  10
#define SERVER_CMD_STOP_RT_CONTROLLER   11
#define SERVER_CMD_GET_STATUS           12
#define SERVER_CMD_DO_CALIBRATION       13
#define SERVER_CMD_GOTO_CARTPOS         14
#define SERVER_CMD_GOTO_JOINTPOS        15
#define SERVER_CMD_JVEL_CTRL            16
#define SERVER_CMD_GRASP_NEEDLE         17
#define SERVER_CMD_RELEASE_NEEDLE       18
#define SERVER_CMD_INSERT_NEEDLE        19
#define SERVER_CMD_DO_SLAVE_MODE        20
#define SERVER_CMD_START_VIRTUAL_ROBOT  21
#define SERVER_CMD_STOP_VIRTUAL_ROBOT   22

#define FIFO_ALREADY_OPEN             1000
#define FIFO_ALREADY_CLOSE            1001
#define OPEN_CMD_FIFO_FAILED          1002
#define OPEN_STATUS_FIFO_FAILED       1003
#define OPEN_ACKNO_FIFO_FAILED        1004
#define OPEN_SAVE_DATA_FIFO_FAILED    1005
#define OPEN_PARAM_FIFO_FAILED        1006
#define OPEN_IDENT_PARAM_FIFO_FAILED    1007


typedef struct{
    double joint_position[NB_JOINTS];
    unsigned int cal_status;
}SERVER_RAW_DATA_STRUCT;

class sRobotController : public QObject
{
    Q_OBJECT
public:
    explicit sRobotController(QObject *parent = 0);
    ~sRobotController();
    void setRefJPos(QVector<double> value);
    void setRefJVel(QVector<double> value);
    QVector<double> getJPos(void){return JPos;}


public slots:
    void            startRTController(void);
    void            stopRTController(void);
    void            open_RTCommunication(void);
    void            close_RTCommunication(void);
    void            doCal(void);
    void            doRefreshStatus(void);
    void            doSaveData(void);
    void            doJVelPlanner(void);
    void            gotoJPosPlanner(void);
    void            gotoCPosPlanner(void);
    void            doIdentification(const IDENT_PARAM_STRUCT &ident_param);
    void            doGraspNeedle(void);
    void            doReleaseNeedle(void);
    void            doSlaveMode(void);
    void            doJPosStatic(void);
    void            ChangeSelectedJoint(int);
    void            readyReadStandardOutput(void);
    void            readyReadStandardError(void);
    void            parserQtControllerServer(QByteArray data);


signals:
    void            ControllerStatusChanged(bool);
    void            RTControllerModeChanged(mode_type);
    void            MeasuresChanged(void);
    void            CalStatusChanged(bool);
    void            CalStepChanged(int);
    void            SecurityStatusChanged(bool);
    void            PowerStatusChanged(bool);
    void            RefreshTimer_stopped(void);
    void            rtController_started(void);
    void            rtFIFO_opened(void);
    void            stopRTProcess();
    void            sendDataToServer(QByteArray data);
    void            virtualRobotStatus(bool);
    void            sendChuckStatus(bool);

protected :
    int             id_fifo_cmd;
    int             id_fifo_ackno;
    int             id_fifo_status;
    int             id_fifo_save_data;
    int             id_fifo_param;
    int             id_fifo_ident_param;

    bool            bFIFOIsOpen;
    unsigned int    controller_mode;
    unsigned int    controller_submode;

    QTimer          *pTimerRefresh;
    QTimer          *pTimerSaveData;
    QProcess        *process_rtController;  // processus permettant l'exécution du controleur temps-réel

    bool            bIsCal;             /// flag indiquant que le robot est étalonné
    bool            bIsRTRun;           /// Flag indiquant que le controleur RT est en cours d'excécution
    bool            bIsPowerOn;         /// flag indiquant que le robot est sous tension
    bool            bIsTeleopOn;
    QVector<double> JPos;               /// Position articulaire courante
    QVector<double> RefJPos;            /// Consigne de position articulaire
    QVector<double> RefCPos;            /// Consigne de Position opérationnelle
    QVector<double> RefJVel;            /// Consigne de vitesse articulaire
    QVector<double> JPosCal;            /// Position articulaire pour l'étalonnage


    QFile           data_file;
    QTextStream     out_data;
    // Fonctions membres protégées
    void send_command(cmd_type new_cmd);
    void open_save_file(void);
    unsigned int selected_joint;        // permet de définir l'actionneur que l'on souhaite controler
    QtControllerServer *m_serveur;
    sRobot *pRobot;
};

#endif // SROBOTCONTROLLER_H
