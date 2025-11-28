#ifndef DIALOGWREN_H
#define DIALOGWREN_H

#include <QDialog>
#include <QTimer>

namespace Ui {
class DialogWren;
}

class DialogWren : public QDialog
{
    Q_OBJECT

    int current_output;
    bool current_output_status;
    double current_pressure;
    double reference_pressure;
    int current_period_ms;
    void sendControl(int,double);
public:
    explicit DialogWren(QWidget *parent = 0);
    ~DialogWren();
    double getPressure(void);
public slots:
    void selectOutput(void);
    void OnBtnStartWrenCtrl(void);
    void OnBtnStopWrenCtrl(void);
    void OnTimeOutCtrlWren(void);
    void changePressure(void);
    void changePeriod(void);
signals:
    void sendOutputStatus(int selected_output, bool status);
    void sendPressureReference(int selected_output,double value);

private:
    Ui::DialogWren *ui;
    QTimer *pTimerCtrlWren;
};

#endif // DIALOGWREN_H
