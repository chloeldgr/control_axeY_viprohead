#include "dialogSPIRITControl.h"
#include "ui_dialogSPIRITControl.h"

DialogSPIRITControl::DialogSPIRITControl(QWidget *parent) :
    QDialog(parent),
    bIsTeleopOn(false),
    ui(new Ui::DialogSPIRITControl)
{
    ui->setupUi(this);
    setTeleopStatus(bIsTeleopOn);
    connect(ui->btn_StartTeleop,            SIGNAL(clicked()),          this,   SLOT(onStartTeleop()));
    connect(ui->btn_StopTeleop,             SIGNAL(clicked()),          this,   SLOT(onStopTeleop()));
    connect(ui->btn_Quit_SPIRITCtrl,        SIGNAL(clicked()),          this,   SLOT(close()));
    connect(ui->hSlider_VPPM1,              SIGNAL(sliderMoved(int)),   this,   SLOT(onRefreshInterface()));
    connect(ui->hSlider_VPPM2,              SIGNAL(sliderMoved(int)),   this,   SLOT(onRefreshInterface()));
    connect(ui->hSlider_SeqPeriod,          SIGNAL(sliderMoved(int)),   this,   SLOT(onRefreshInterface()));
    connect(ui->hSlider_GripperPeriod,      SIGNAL(sliderMoved(int)),   this,   SLOT(onRefreshInterface()));
}


DialogSPIRITControl::~DialogSPIRITControl()
{
    delete ui;
}


void DialogSPIRITControl::onStartTeleop(void)
{
    VPPM1_Voltage   = (double)(ui->hSlider_VPPM1->value())*10/6000;
    VPPM2_Voltage   = (double)(ui->hSlider_VPPM2->value())*10/6000;
    SeqPeriod       = (double)(ui->hSlider_SeqPeriod->value());
    GripperPeriod   = (double)(ui->hSlider_GripperPeriod->value());
    int direction;

    Qt::CheckState direction_state =  ui->cb_actuation_direction->checkState();
    switch(direction_state)
    {
        case Qt::Unchecked:
            direction = -1;
            break;
    case Qt::Checked:
            direction = 1;
            break;
    default :
        direction = 1;
        break;
    }

    setTeleopStatus(true);
    emit startTeleop( direction,
                      VPPM1_Voltage,
                      VPPM2_Voltage,
                      SeqPeriod,
                      GripperPeriod);
}
void DialogSPIRITControl::onStopTeleop(void)
{
    setTeleopStatus(false);
    emit stopTeleop();
}


void DialogSPIRITControl::setTeleopStatus(bool state)
{
    bIsTeleopOn = state;
    if ( !bIsTeleopOn )
    {
        ui->btn_StartTeleop->setCheckable(true);
        ui->btn_StopTeleop->setCheckable(false);
    }
    else
    {
        ui->btn_StartTeleop->setCheckable(false);
        ui->btn_StopTeleop->setCheckable(true);
    }

}


void DialogSPIRITControl::setOrientationLED(bool state)
{
    ui->Orientation_LED->setChecked(state);
}

void DialogSPIRITControl::setInsertionLED(bool state)
{
    ui->Insertion_LED->setChecked(state);
}

void DialogSPIRITControl::onRefreshInterface(void)
{


    ui->lEdit_VPPM1_Pressure->setText(QString::number(ui->hSlider_VPPM1->value()));
    ui->lEdit_VPPM2_Pressure->setText(QString::number(ui->hSlider_VPPM2->value()));
    ui->lineEdit_Te->setText(QString::number(ui->hSlider_SeqPeriod->value()));
    ui->lineEdit_Tp->setText(QString::number(ui->hSlider_GripperPeriod->value()));

    VPPM1_Voltage   = (double)(ui->hSlider_VPPM1->value())*10/6000;
    VPPM2_Voltage   = (double)(ui->hSlider_VPPM2->value())*10/6000;
    SeqPeriod       = (double)(ui->hSlider_SeqPeriod->value());
    GripperPeriod   = (double)(ui->hSlider_GripperPeriod->value());
    int direction;

    Qt::CheckState direction_state =  ui->cb_actuation_direction->checkState();
    switch(direction_state)
    {
        case Qt::Unchecked:
            direction = -1;
            break;
    case Qt::Checked:
            direction = 1;
            break;
    default :
        direction = 1;
        break;
    }
           emit sendParam(502, //control insertion
                        direction,
                        VPPM1_Voltage,
                        VPPM2_Voltage,
                        SeqPeriod,
                        GripperPeriod);


}
