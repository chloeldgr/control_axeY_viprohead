#include "DialogWren.h"
#include "ui_DialogWren.h"
#include <QDebug>
using namespace std;

DialogWren::DialogWren(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogWren),
    pTimerCtrlWren(new QTimer)
{
    ui->setupUi(this);
    current_output = 0;
    reference_pressure = 0;
    current_pressure = 0;
    current_period_ms = 1000;

    ui->lineEdit_Periode->setText(QString::number(current_period_ms));
    ui->lineEdit_Pressure->setText(QString::number(reference_pressure));

    pTimerCtrlWren->setInterval(current_period_ms);
    connect(pTimerCtrlWren,         SIGNAL(timeout()),          this,   SLOT(OnTimeOutCtrlWren()));
    connect(ui->checkBox_S1,        SIGNAL(clicked(bool)),      this,   SLOT(selectOutput()));
    connect(ui->checkBox_S2,        SIGNAL(clicked(bool)),      this,   SLOT(selectOutput()));
    connect(ui->checkBox_S3,        SIGNAL(clicked(bool)),      this,   SLOT(selectOutput()));
    connect(ui->checkBox_S4,        SIGNAL(clicked(bool)),      this,   SLOT(selectOutput()));
    connect(ui->btn_start,          SIGNAL(clicked(bool)),      this,   SLOT(OnBtnStartWrenCtrl()));
    connect(ui->btn_stop,           SIGNAL(clicked(bool)),      this,   SLOT(OnBtnStopWrenCtrl()));
    connect(ui->lineEdit_Pressure,  SIGNAL(returnPressed()),    this,   SLOT(changePressure()));
    connect(ui->lineEdit_Periode,   SIGNAL(returnPressed()),    this,   SLOT(changePeriod()));

}

DialogWren::~DialogWren()
{
    delete ui;
}

///
/// \fn DialogWren::changePressure
/// \brief changement de la pression courante
///
void DialogWren::changePressure(void)
{
    reference_pressure = ui->lineEdit_Pressure->text().toDouble();
    if (reference_pressure > 6.0)
    {
        reference_pressure = 6.0;
            ui->lineEdit_Pressure->setText(QString::number(reference_pressure));
    }
   emit  sendPressureReference(current_output,reference_pressure);
}

///
/// \fn DialogWren::changePeriod
/// \brief Changement de la période de la boucle
///
void DialogWren::changePeriod(void)
{
    current_period_ms = ui->lineEdit_Periode->text().toDouble();
    pTimerCtrlWren->setInterval(current_period_ms);

}



void DialogWren::OnTimeOutCtrlWren(void)
{

    if ( !current_output_status)
        current_output_status = true;
    else
        current_output_status = false;

    reference_pressure = ui->lineEdit_Pressure->text().toDouble();


    //qDebug() << "Send Ouput Stauts " << current_output << " " <<current_output_status << " P = " << reference_pressure;
    emit sendOutputStatus(current_output,current_output_status);
    //emit sendPressureReference(current_output,reference_pressure);

}


///
/// \fn DialogWren::selectOutput
/// \brief sélection de la sortie
///
void DialogWren::selectOutput(void)
{
    if ( ui->checkBox_S1->isChecked())
    {
        current_output = 1;
        ui->checkBox_S2->setChecked(false);
        ui->checkBox_S3->setChecked(false);
        ui->checkBox_S4->setChecked(false);
    }
    if ( ui->checkBox_S2->isChecked())
    {
        current_output = 2;
        ui->checkBox_S1->setChecked(false);
        ui->checkBox_S3->setChecked(false);
        ui->checkBox_S4->setChecked(false);
    }
    if ( ui->checkBox_S3->isChecked())
    {
        current_output = 3;
        ui->checkBox_S2->setChecked(false);
        ui->checkBox_S1->setChecked(false);
        ui->checkBox_S4->setChecked(false);
    }
    if ( ui->checkBox_S4->isChecked())
    {
        current_output = 4;

        ui->checkBox_S2->setChecked(false);
        ui->checkBox_S3->setChecked(false);
        ui->checkBox_S1->setChecked(false);
    }

}



void DialogWren::OnBtnStartWrenCtrl(void)
{
    selectOutput();
    if (current_output>0) // si aucune sorties n'a été sléectionnée alors on envoi rien
    {
        // Lecture de la commande à envoyer à la servovanne
        reference_pressure = ui->lineEdit_Pressure->text().toDouble();
        current_period_ms = ui->lineEdit_Periode->text().toDouble();
        emit sendPressureReference(current_output,reference_pressure);

        pTimerCtrlWren->setInterval(current_period_ms);

        // si l'utilisateur a spécifié qu'il veut envoyer un signal périodique alors
        if (ui->cb_periodic_status->checkState() == Qt::Checked)
        {
            qDebug() << "Demarrage sequence periodique.";
            pTimerCtrlWren->start();
        }
        else
        {
            // sinon envoi de la commande directement
            emit sendOutputStatus(current_output,true);
        }
    }
}


void DialogWren::OnBtnStopWrenCtrl(void)
{
    pTimerCtrlWren->stop();

    reference_pressure = 0;
    emit sendPressureReference(current_output,reference_pressure);
    emit sendOutputStatus(current_output, false);
    ui->lineEdit_Pressure->setText(QString::number(reference_pressure));
    current_output = 0;
    ui->checkBox_S1->setChecked(false);
    ui->checkBox_S2->setChecked(false);
    ui->checkBox_S3->setChecked(false);
    ui->checkBox_S4->setChecked(false);

}
