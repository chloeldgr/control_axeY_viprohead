#include "DialogInchWormActuator.h"
#include "ui_DialogInchWormActuator.h"

DialogInchWormActuator::DialogInchWormActuator(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogInchWormActuator)
{
    ui->setupUi(this);

    ui->lEdit_VPPM1_Pressure->setText(QString::number(ui->hSlider_VPPM1->value()));
    ui->lEdit_VPPM2_Pressure->setText(QString::number(ui->hSlider_VPPM2->value()));
    ui->lineEdit_Te->setText(QString::number(ui->hSlider_SeqPeriod->value()));
    ui->lineEdit_Tp->setText(QString::number(ui->hSlider_GripperPeriod->value()));

    connect(ui->BtnStartInchworm,                   SIGNAL(clicked()),          this,   SLOT(OnBtnStartInchworm()));

    connect(ui->BtnStopInchworm,                    SIGNAL(clicked()),          this,   SLOT(OnBtnStopInchworm()));
    connect(ui->btn_Quit,                            SIGNAL(clicked()),          this,   SLOT(OnBtnQuit()));

    connect(ui->hSlider_VPPM1,                      SIGNAL(sliderMoved(int)),   this,   SLOT(OnRefreshInterface()));
    connect(ui->hSlider_VPPM2,                      SIGNAL(sliderMoved(int)),   this,   SLOT(OnRefreshInterface()));
    connect(ui->hSlider_SeqPeriod,                  SIGNAL(sliderMoved(int)),   this,   SLOT(OnRefreshInterface()));
    connect(ui->hSlider_GripperPeriod,              SIGNAL(sliderMoved(int)),   this,   SLOT(OnRefreshInterface()));
    connect(ui->cb_actuation_direction,             SIGNAL(clicked()),          this,   SLOT(OnRefreshInterface()));



}

DialogInchWormActuator::~DialogInchWormActuator()
{

    delete ui;
}




void DialogInchWormActuator::OnRefreshInterface(void)
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


void DialogInchWormActuator::OnBtnStartInchworm(void)
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
           emit sendParam(501, //control insertion
                        direction,
                        VPPM1_Voltage,
                        VPPM2_Voltage,
                        SeqPeriod,
                        GripperPeriod);

}

void DialogInchWormActuator::OnBtnStopInchworm(void)
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
           emit sendParam(503, //control insertion
                        direction,
                        VPPM1_Voltage,
                        VPPM2_Voltage,
                        SeqPeriod,
                        GripperPeriod);


}

void DialogInchWormActuator::OnBtnQuit(void)
{

    OnBtnStopInchworm();

    close();

}
