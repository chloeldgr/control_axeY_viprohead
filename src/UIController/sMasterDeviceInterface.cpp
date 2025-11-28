#include "sMasterDeviceInterface.h"
#include "ui_MasterDeviceInterface.h"

MasterDeviceInterface::MasterDeviceInterface(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MasterDeviceInterface),
    pMasterRobot(new sMasterRobot)
{
    ui->setupUi(this);

    connect(ui->btn_ConnectRobot,       SIGNAL(clicked(bool)),                      pMasterRobot,                   SLOT(startRTController()));
    connect(ui->btn_DisconnectRobot,    SIGNAL(clicked(bool)),                      pMasterRobot,                   SLOT(stopRTController()));
    connect(ui->btn_StartTeleop,        SIGNAL(clicked(bool)),                      this,                           SLOT(startTeleop()));
    connect(ui->btn_StopTeleop,         SIGNAL(clicked(bool)),                      this,                           SLOT(stopTeleop()));
    connect(pMasterRobot,               SIGNAL(RTControllerModeChanged(QString)),   ui->edit_RobotControllerMode,   SLOT(setText(QString)));
    connect(pMasterRobot,               SIGNAL(MeasuresChanged(QString)),           ui->edit_JPos,                  SLOT(setText(QString)));
}

MasterDeviceInterface::~MasterDeviceInterface()
{
    delete ui;
}

///
/// \brief MasterDeviceInterface::startTeleop
///
void MasterDeviceInterface::startTeleop()
{
   pMasterRobot->doStartTeleop();
   emit teleopStarted();
}

///
/// \brief MasterDeviceInterface::stopTeleop
///
void MasterDeviceInterface::stopTeleop()
{
    pMasterRobot->doStopTeleop();
    emit teleopStopped();
}

