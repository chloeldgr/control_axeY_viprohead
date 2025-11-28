
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include "../rtController/rtController.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    pController(new sRTController),
    pEtherCATMaster(new sEtherCAT),
    pDialogInchworm(new DialogInchWormActuator),
    pDialogFlatInchworm(new DialogFlatInchwormCtrl),
    pDialogSPIRITControl(new DialogSPIRITControl),
    pDialogAdvancedControl(new DialogAvancedControl),
    pDialogWren(new DialogWren)
{

    int i;

    ui->setupUi(this);

    createActions();
    createMenus();

    // Mise en forme des sliders pour la commande des sorties analogiques
    ui->horizontalSlider_ANA_OUT_1->setMaximum(10000);
    ui->horizontalSlider_ANA_OUT_1->setMinimum(0);
    ui->horizontalSlider_ANA_OUT_1->setTickInterval(1);
    ui->lineEdit_ANA_OUT_1->setText(QString::number(ui->horizontalSlider_ANA_OUT_1->value()));

    ui->horizontalSlider_ANA_OUT_2->setMaximum(10000);
    ui->horizontalSlider_ANA_OUT_2->setMinimum(0);
    ui->horizontalSlider_ANA_OUT_2->setTickInterval(1);
    ui->lineEdit_ANA_OUT_2->setText(QString::number(ui->horizontalSlider_ANA_OUT_2->value()));

    ui->horizontalSlider_ANA_OUT_3->setMaximum(10000);
    ui->horizontalSlider_ANA_OUT_3->setMinimum(0);
    ui->horizontalSlider_ANA_OUT_3->setTickInterval(1);
    ui->lineEdit_ANA_OUT_3->setText(QString::number(ui->horizontalSlider_ANA_OUT_3->value()));

    ui->horizontalSlider_ANA_OUT_4->setMaximum(10000);
    ui->horizontalSlider_ANA_OUT_4->setMinimum(0);
    ui->horizontalSlider_ANA_OUT_4->setTickInterval(1);
    ui->lineEdit_ANA_OUT_4->setText(QString::number(ui->horizontalSlider_ANA_OUT_4->value()));

    ui->horizontalSlider_ANA_OUT_5->setMaximum(5000);
    ui->horizontalSlider_ANA_OUT_5->setMinimum(0);
    ui->horizontalSlider_ANA_OUT_5->setTickInterval(1);
    ui->horizontalSlider_ANA_OUT_5->setValue(2500);
    ui->lineEdit_ANA_OUT_5->setText(QString::number(ui->horizontalSlider_ANA_OUT_5->value()));

    ui->horizontalSlider_ANA_OUT_6->setMaximum(5000);
    ui->horizontalSlider_ANA_OUT_6->setMinimum(0);
    ui->horizontalSlider_ANA_OUT_6->setValue(2500);

    ui->horizontalSlider_ANA_OUT_6->setTickInterval(1);
    ui->lineEdit_ANA_OUT_6->setText(QString::number(ui->horizontalSlider_ANA_OUT_6->value()));

    // Mise en forme de l'affichage des entrées analogiques
    ui->lineEdit_ANA_IN_1->setText(QString::number(0));
    ui->lineEdit_ANA_IN_2->setText(QString::number(0));
    ui->lineEdit_ANA_IN_3->setText(QString::number(0));
    ui->lineEdit_ANA_IN_4->setText(QString::number(0));
    ui->lineEdit_Pressure_Sensor_S1->setText(QString::number(0));
    ui->lineEdit_Pressure_Sensor_S2->setText(QString::number(0));
    ui->lineEdit_Pressure_Sensor_S3->setText(QString::number(0));
    ui->lineEdit_Pressure_Sensor_S4->setText(QString::number(0));
    ui->lineEdit_ANA_IN_9->setText(QString::number(0));
    ui->lineEdit_ANA_IN_10->setText(QString::number(0));
    ui->lineEdit_ANA_IN_11->setText(QString::number(0));
    ui->lineEdit_ANA_IN_12->setText(QString::number(0));
    ui->lineEdit_ANA_IN_13->setText(QString::number(0));
    ui->lineEdit_ANA_IN_14->setText(QString::number(0));
    ui->lineEdit_ANA_IN_15->setText(QString::number(0));

    ui->power_led->setDisabled(true);
    ui->power_led->setChecked(false);
    ui->security_led_2->setDisabled(true);
    ui->security_led_2->setChecked(false);
    ui->calibration_led->setDisabled(true);
    ui->calibration_led->setChecked(false);
    ui->EtherCAT_led_status->setDisabled(true);
    ui->EtherCAT_led_status->setChecked(false);

    struct sigaction usr;
    usr.sa_handler =  sRTController::Usr1SignalHandler;
    sigemptyset(&usr.sa_mask);
    usr.sa_flags = 0;
    usr.sa_flags|= SA_RESTART;
    if ( sigaction(SIGUSR1, &usr,0) > 0)
        return;
    for (i = 0; i < 6; i ++)
        analog_value[i] = 0;
}

MainWindow::~MainWindow()
{

    //delete master_controller;
    delete ui;
}





void MainWindow::about()
{

    QMessageBox::about(this, tr("Pneumatic workbench"),
            tr("xxxx"));
}


void MainWindow::createActions()
{
    connect(ui->btn_Quit,                   SIGNAL(clicked()),                  this,           SLOT(close()));
    connect(ui->btn_setDigOutput,           SIGNAL(clicked()),                  this,           SLOT(setDigitalOutput()));
    connect(ui->btn_setDigOutput_EV,        SIGNAL(clicked()),                  this,           SLOT(setDigOutEV()));
   // connect(ui->btn_setDigOutput5V,         SIGNAL(clicked()),                  this,           SLOT(setDigOut5V()));
    connect(this,                           SIGNAL(changeDigitalOuput(int)),    pController,    SLOT(doWriteDigOut(int)));
    connect(ui->horizontalSlider_ANA_OUT_1, SIGNAL(sliderMoved(int)),           this,           SLOT(UpdateDisplay()));
    connect(ui->horizontalSlider_ANA_OUT_2, SIGNAL(sliderMoved(int)),           this,           SLOT(UpdateDisplay()));
    connect(ui->horizontalSlider_ANA_OUT_3, SIGNAL(sliderMoved(int)),           this,           SLOT(UpdateDisplay()));
    connect(ui->horizontalSlider_ANA_OUT_4, SIGNAL(sliderMoved(int)),           this,           SLOT(UpdateDisplay()));
    connect(ui->horizontalSlider_ANA_OUT_5, SIGNAL(sliderMoved(int)),           this,           SLOT(UpdateDisplay()));
    connect(ui->horizontalSlider_ANA_OUT_6, SIGNAL(sliderMoved(int)),           this,           SLOT(UpdateDisplay()));

    connect(pController,                    SIGNAL(MeasuresChanged(double*)),       this,           SLOT(UpdateAnalogInput(double*)));

    connect(pDialogWren,                    SIGNAL(sendPressureReference(int,double)),  this,           SLOT(CtrlWrenPressure(int,double)));
    connect(pDialogWren,                    SIGNAL(sendOutputStatus(int,bool)),         this,           SLOT(CtrlWrenOutput(int,bool)));
    // Gestion des signaux provenants de la boite de dialogue  qui permet de controler le robot SPIRIT
    connect(pDialogSPIRITControl,           SIGNAL(startTeleop(int,double,double,double,double)),             pController,    SLOT(startTeleop(int,double,double,double,double)));
    connect(pDialogSPIRITControl,           SIGNAL(stopTeleop()),             pController,    SLOT(stopTeleop()));


    connect(pDialogInchworm,        SIGNAL(sendParam(int,int,double, double, double, double)),      pController,    SLOT(doCtrlAuxeticInchworm(int,int,double,double,double,double)));
    connect(pDialogFlatInchworm,    SIGNAL(start(int,double,bool)),                             pController,    SLOT(doCtrlFlatInchworm(int, double,bool)));
   // connect(pDialogFlatInchworm,    SIGNAL(stop()),                                             pController,    SLOT(doStopInchworm()));


    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    exitAct->setStatusTip(tr("Exit the application"));


    openEtherCATAct = new QAction(tr("Open EtherCAT Communication"), this);
    openEtherCATAct->setStatusTip("Open EtherCAT Communication");

    closeEtherCATAct = new QAction(tr("Close EtherCAT Communication"), this);
    closeEtherCATAct->setStatusTip("Close EtherCAT Communication");

    startRTControllerAct = new QAction(tr("Start RT Controller"), this);
    startRTControllerAct->setStatusTip("Start Slave Controller");

    stopRTControllerAct = new QAction(tr("Stop RT Controller"), this);
    stopRTControllerAct->setStatusTip("Stop Slave Controller");

    aboutAct = new QAction(tr("&About"), this);
    aboutAct->setStatusTip(tr("Show the application's About box"));

    inchwormAct = new QAction(tr("Auxetic InchWorm"), this);
    inchwormAct->setStatusTip(tr("Auxetic Inchworm Actuator Controller"));

    flatinchwormAct = new QAction(tr("Flat InchWorm"), this);
    flatinchwormAct->setStatusTip(tr("Flat Inchworm Actuator Controller"));

    Open_dialogSPIRITControl = new QAction(tr("SPIRIT Control"), this);
    Open_dialogSPIRITControl->setStatusTip(tr("SPIRIT motion control"));

    Open_DialogAdvancedControl = new QAction(tr("Advanced Control"), this);
    Open_DialogAdvancedControl->setStatusTip(tr("Advanced control"));

    Open_DialogWren = new QAction(tr("Wren Control"), this);
    Open_DialogWren->setStatusTip(tr("Wren Control"));

    connect(aboutAct,                   SIGNAL(triggered()),                        this,                   SLOT(about()));
    connect(exitAct,                    SIGNAL(triggered()),                        this,                   SLOT(close()));
    connect(openEtherCATAct,            SIGNAL(triggered()),                        pEtherCATMaster,        SLOT(open()));
    connect(closeEtherCATAct,           SIGNAL(triggered()),                        pEtherCATMaster,        SLOT(close()));
    connect(startRTControllerAct,       SIGNAL(triggered()),                        pController,            SLOT(start()));
    connect(stopRTControllerAct,        SIGNAL(triggered()),                        pController,            SLOT(stop()));
    connect(inchwormAct,                SIGNAL(triggered()),                        pDialogInchworm,        SLOT(open()));
    connect(flatinchwormAct,            SIGNAL(triggered()),                        pDialogFlatInchworm,    SLOT(open()));
    connect(Open_dialogSPIRITControl,   SIGNAL(triggered()),                        pDialogSPIRITControl,   SLOT(open()));
    connect(Open_DialogAdvancedControl, SIGNAL(triggered()),                        pDialogAdvancedControl, SLOT(open()));
    connect(Open_DialogWren,            SIGNAL(triggered()),                        pDialogWren,            SLOT(open()));
    connect(pEtherCATMaster,            SIGNAL(ECMasterStatusChanged()),            this,                   SLOT(ChangeECMasterStatus()));


    connect(pController,                SIGNAL(ControllerStatusChanged(bool)),      ui->led_controller,     SLOT(setChecked(bool)));
    connect(pController,                SIGNAL(CalStatusChanged(bool)),             ui->calibration_led,    SLOT(setChecked(bool)));
    connect(pController,                SIGNAL(PowerStatusChanged(bool)),           ui->power_led,          SLOT(setChecked(bool)));
  // connect(pController,                SIGNAL(GraspingButtonActivated(bool)),          this,                   SLOT(CtrlGrasping(bool)));
}

void MainWindow::createMenus()
{
    mainAppMenu = menuBar()->addMenu(tr("App"));
    mainAppMenu->addAction(exitAct);

    EtherCATMenu = menuBar()->addMenu(tr("EtherCAT Bus"));
    EtherCATMenu->addAction(openEtherCATAct);
    EtherCATMenu->addAction(closeEtherCATAct);

    SlaveRTControllerMenu = menuBar()->addMenu(tr("Slave RT Controller"));
    SlaveRTControllerMenu->addAction(startRTControllerAct);
    SlaveRTControllerMenu->addAction(stopRTControllerAct);

    RobotControlMenu = menuBar()->addMenu(tr("Robot Action"));
    RobotControlMenu->addAction(inchwormAct);
    RobotControlMenu->addAction(flatinchwormAct);
    RobotControlMenu->addAction(Open_dialogSPIRITControl);
    RobotControlMenu->addAction(Open_DialogAdvancedControl);
    RobotControlMenu->addAction(Open_DialogWren);
//    RobotControlMenu->addAction(closeChuckAct);

    helpMenu = menuBar()->addMenu(tr("Help"));
    helpMenu->addAction(aboutAct);


}



void MainWindow::ChangeECMasterStatus(void)
{
    // Un changement d'état du Master EtherCAT a été détecté
    ui->EtherCAT_led_status->setChecked(pEtherCATMaster->getECMasterStatus());
    // Si EC Master est running alors on autorise l'utilisateur à lancer RT Controller
    startRTControllerAct->setDisabled(!pEtherCATMaster->getECMasterStatus());
    stopRTControllerAct->setDisabled(!pEtherCATMaster->getECMasterStatus());

}

void MainWindow::CtrlWrenPressure(int output, double pressure)
{
    int i;


    if (output == 0)
            analog_value[0] = pressure*1666.7;
    if (output ==1)
        analog_value[0] = pressure*1666.7;
    if (output ==2)
        analog_value[0] = pressure*1666.7;

    if (output ==3)
        analog_value[1] = pressure*1666.7;

    if (output ==4)
        analog_value[1] = pressure*1666.7;


    qDebug() << "Ouput : " << output << " analog value " << analog_value[0];
    pController->changeAnalogOutput(analog_value);


}


void MainWindow::CtrlWrenOutput(int output, bool status)
{
    std::bitset<16> reg;
    int i;
    for (i=0; i < 16;i++)
        reg.set(i,  0);

        reg.set(output-1,  status);

    emit changeDigitalOuput((int)(reg.to_ulong()));

}


void MainWindow::CtrlGrasping(bool status)
{
    int analog_value[6];

    analog_value[0] = ui->horizontalSlider_ANA_OUT_1->value();
    analog_value[1] = ui->horizontalSlider_ANA_OUT_2->value();
    analog_value[2] = ui->horizontalSlider_ANA_OUT_3->value();
    analog_value[3] = ui->horizontalSlider_ANA_OUT_4->value();
    analog_value[4] = ui->horizontalSlider_ANA_OUT_5->value();
    analog_value[5] = ui->horizontalSlider_ANA_OUT_6->value();

    pController->changeAnalogOutput(analog_value);
    // Récupération des états des CheckBox associés aux sorties TOR
    std::bitset<16> reg;

    reg.set(0,  ui->checkBox_DOUT_1->isChecked());
    reg.set(1,  ui->checkBox_DOUT_2->isChecked());
    reg.set(2,  status);
    reg.set(3,  ui->checkBox_DOUT_4->isChecked());
    reg.set(4,  0);
    reg.set(5,  0);
    reg.set(6,  0);
    reg.set(7,  0);
    reg.set(8,  0);
    reg.set(9,  0);
    reg.set(10, 0);
    reg.set(11, 0);
    reg.set(12, 0);
    reg.set(13, 0);
    reg.set(14, 0);
    reg.set(15, 0);

    emit changeDigitalOuput((int)(reg.to_ulong()));

}

void MainWindow::UpdateAnalogInput(double *value)
{
    ui->lineEdit_ANA_IN_1->setText(QString::number((int)(value[0]*1000),'f',0));
    ui->lineEdit_ANA_IN_2->setText(QString::number((int)(value[1]*1000),'f',0));
    ui->lineEdit_ANA_IN_3->setText(QString::number((int)(value[2]*1000),'f',0));
    ui->lineEdit_ANA_IN_4->setText(QString::number((int)(value[3]*1000),'f',0));
    ui->lineEdit_ANA_IN_9->setText(QString::number((int)(value[8]*1000),'f',0));
    ui->lineEdit_ANA_IN_10->setText(QString::number((int)(value[9]*1000),'f',0));
    ui->lineEdit_ANA_IN_11->setText(QString::number((int)(value[10]*1000),'f',0));
    ui->lineEdit_ANA_IN_12->setText(QString::number((int)(value[11]*1000),'f',0));
    ui->lineEdit_ANA_IN_13->setText(QString::number((int)(value[12]*1000),'f',0));
    ui->lineEdit_ANA_IN_14->setText(QString::number((int)(value[13]*1000),'f',0));
    ui->lineEdit_ANA_IN_15->setText(QString::number((int)(value[14]*1000),'f',0));
    ui->lineEdit_ANA_IN_16->setText(QString::number((int)(value[15]*1000),'f',0));
//    ui->lineEdit_ANA_IN_17->setText(QString::number((int)(value[16]*1000),'f',0));
//    ui->lineEdit_ANA_IN_18->setText(QString::number((int)(value[17]*1000),'f',0));
//    ui->lineEdit_ANA_IN_19->setText(QString::number((int)(value[18]*1000),'f',0));
//    ui->lineEdit_ANA_IN_20->setText(QString::number((int)(value[19]*1000),'f',0));
    ui->lineEdit_Pressure_Sensor_S1->setText(QString::number((int)(value[4]*1000),'f',0));
    ui->lineEdit_Pressure_Sensor_S2->setText(QString::number((int)(value[5]*1000),'f',0));
    ui->lineEdit_Pressure_Sensor_S3->setText(QString::number((int)(value[6]*1000),'f',0));
    ui->lineEdit_Pressure_Sensor_S4->setText(QString::number((int)(value[7]*1000),'f',0));
}


void MainWindow::UpdateDisplay(void)
{
    int analog_value[6];
    ui->lineEdit_ANA_OUT_1->setText(QString::number(ui->horizontalSlider_ANA_OUT_1->value()));
    ui->lineEdit_ANA_OUT_2->setText(QString::number(ui->horizontalSlider_ANA_OUT_2->value()));
    ui->lineEdit_ANA_OUT_3->setText(QString::number(ui->horizontalSlider_ANA_OUT_3->value()));
    ui->lineEdit_ANA_OUT_4->setText(QString::number(ui->horizontalSlider_ANA_OUT_4->value()));
    ui->lineEdit_ANA_OUT_5->setText(QString::number(ui->horizontalSlider_ANA_OUT_5->value()));
    ui->lineEdit_ANA_OUT_6->setText(QString::number(ui->horizontalSlider_ANA_OUT_6->value()));
    analog_value[0] = ui->horizontalSlider_ANA_OUT_1->value();
    analog_value[1] = ui->horizontalSlider_ANA_OUT_2->value();
    analog_value[2] = ui->horizontalSlider_ANA_OUT_3->value();
    analog_value[3] = ui->horizontalSlider_ANA_OUT_4->value();
    analog_value[4] = ui->horizontalSlider_ANA_OUT_5->value();
    analog_value[5] = ui->horizontalSlider_ANA_OUT_6->value();

    pController->changeAnalogOutput(analog_value);
}

void MainWindow::ChangeSlaveControllerMode(mode_type new_mode)
{
    qDebug() << "Slave Controller Mode "<< new_mode;

}




void MainWindow::setDigitalOutput(void)
{
    // Récupération des états des CheckBox associés aux sorties TOR
    std::bitset<16> reg;

    reg.set(0,  0);
    reg.set(1,  0);
    reg.set(2,  0);
    reg.set(3,  0);
    reg.set(4,  ui->checkBox_DOUT_5->isChecked());
    reg.set(5,  ui->checkBox_DOUT_6->isChecked());
    reg.set(6,  ui->checkBox_DOUT_7->isChecked());
    reg.set(7,  ui->checkBox_DOUT_8->isChecked());
    reg.set(8,  ui->checkBox_DOUT_9->isChecked());
    reg.set(9,  ui->checkBox_DOUT_10->isChecked());
    reg.set(10, ui->checkBox_DOUT_11->isChecked());
    reg.set(11, ui->checkBox_DOUT_12->isChecked());
    reg.set(12, 0);
    reg.set(13, 0);
    reg.set(14, 0);
    reg.set(15, 0);

    emit changeDigitalOuput((int)(reg.to_ulong()));


}
void MainWindow::setDigOutEV(void)
{
    // Récupération des états des CheckBox associés aux sorties TOR
    std::bitset<16> reg;

    reg.set(0,  ui->checkBox_DOUT_1->isChecked());
    reg.set(1,  ui->checkBox_DOUT_2->isChecked());
    reg.set(2,  ui->checkBox_DOUT_3->isChecked());
    reg.set(3,  ui->checkBox_DOUT_4->isChecked());
    reg.set(4,  0);
    reg.set(5,  0);
    reg.set(6,  0);
    reg.set(7,  0);
    reg.set(8,  0);
    reg.set(9,  0);
    reg.set(10, 0);
    reg.set(11, 0);
    reg.set(12, 0);
    reg.set(13, 0);
    reg.set(14, 0);
    reg.set(15, 0);

    emit changeDigitalOuput((int)(reg.to_ulong()));


}

//void MainWindow::setDigOut5V(void)
//{
//    // Récupération des états des CheckBox associés aux sorties TOR
//    std::bitset<16> reg;

//    reg.set(0,  ui->checkBox_DOUT_16->isChecked());
//    reg.set(1,  ui->checkBox_DOUT_17->isChecked());
//    reg.set(2,  ui->checkBox_DOUT_18->isChecked());
//    reg.set(3,  ui->checkBox_DOUT_19->isChecked());
//    reg.set(4,  0);
//    reg.set(5,  0);
//    reg.set(6,  0);
//    reg.set(7,  0);
//    reg.set(8,  0);
//    reg.set(9,  0);
//    reg.set(10, 0);
//    reg.set(11, 0);
//    reg.set(12, 0);
//    reg.set(13, 0);
//    reg.set(14, 0);
//    reg.set(15, 0);

//    pController->doWriteDigOut5V((int)(reg.to_ulong()));

//}
