#ifndef DIALOGINCHWORMACTUATOR_H
#define DIALOGINCHWORMACTUATOR_H

#include <QDialog>
#include <QTimer>
#include <QDebug>

namespace Ui {
class DialogInchWormActuator;
}


class DialogInchWormActuator : public QDialog
{
    Q_OBJECT

public:
    explicit DialogInchWormActuator(QWidget *parent = 0);
    ~DialogInchWormActuator();
    double  getVPPM1_Pressure(void);
    double  getVPPM2_Pressure(void);
    double  getInchwormPeriod(void);
    double  getChuckCloseTime(void);
    enum AUXETIC_INCHWORM_CMD{
        START_CTRL=0,
        STOP_CTRL=1,
        REFRESH_CTRL=2
    };
    double VPPM1_Voltage;
    double VPPM2_Voltage;
    double SeqPeriod;
    double GripperPeriod;
private slots:
    void OnBtnStopInchworm(void);
    void OnBtnStartInchworm(void);
    void OnRefreshInterface(void);
    void OnBtnQuit(void);
signals:
    void sendParam(     int,
                        int,
                        double,
                        double,
                        double,
                        double);


private:
    Ui::DialogInchWormActuator *ui;

};

#endif // DIALOGINCHWORMACTUATOR_H
