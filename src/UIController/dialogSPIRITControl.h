#ifndef DIALOGSPIRITCONTROL_H
#define DIALOGSPIRITCONTROL_H

#include <QDialog>

namespace Ui {
class DialogSPIRITControl;
}

class DialogSPIRITControl : public QDialog
{
    Q_OBJECT
    bool bIsTeleopOn;
    double VPPM1_Voltage;
    double VPPM2_Voltage;
    double SeqPeriod;
    double GripperPeriod;
public:
    explicit DialogSPIRITControl(QWidget *parent = 0);
    ~DialogSPIRITControl();
    void setTeleopStatus(bool state);
    void setOrientationLED(bool state);
    void setInsertionLED(bool state);

private slots:
    void onStartTeleop(void);
    void onStopTeleop(void);
    void onRefreshInterface(void);
signals:
    void startTeleop(int,double,double,double,double);//void);
    void stopTeleop();
    void sendParam(     int,
                        int,
                        double,
                        double,
                        double,
                        double);
private:

    Ui::DialogSPIRITControl *ui;
};

#endif // DIALOGSPIRITCONTROL_H
