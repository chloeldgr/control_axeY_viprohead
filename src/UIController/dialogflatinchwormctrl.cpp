#include "dialogflatinchwormctrl.h"
#include "ui_dialogflatinchwormctrl.h"

DialogFlatInchwormCtrl::DialogFlatInchwormCtrl(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogFlatInchwormCtrl)
{
    ui->setupUi(this);
    connect(ui->btn_start,  SIGNAL(clicked()), this, SLOT(onBtnStart()));
    connect(ui->btn_stop,   SIGNAL(clicked()), this, SIGNAL(stop()));
}

DialogFlatInchwormCtrl::~DialogFlatInchwormCtrl()
{
    delete ui;
}


void DialogFlatInchwormCtrl::onBtnStart(void)
{
    int dc = ui->spinBox_duty_cycle_flatinchworm->value();
    double frequency = ui->lineEdit_frequency_flat_inchworm->text().toDouble();
    bool direction = ui->checkBox_Inchworm_direction->isChecked();
    qDebug() << dc << " "<< frequency<< " "<< direction;
    emit start(dc,frequency,direction);
}
