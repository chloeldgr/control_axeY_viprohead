#ifndef DIALOGADVANCEDCONTROL_H
#define DIALOGADVANCEDCONTROL_H

#include <QDialog>
#include "sRTController.h"

namespace Ui {
class DialogAvancedControl;
}

class DialogAvancedControl : public QDialog
{
    Q_OBJECT


    std::list<ADVANCED_DIRECT_CONTROL_STRUCT> listOfDirectControlData;

public:
    explicit DialogAvancedControl(QWidget *parent = 0);
    ~DialogAvancedControl();
signals:


private slots:
    void on_btn_LoadData_clicked();


    void on_btn_SendData_clicked();

    void on_btn_Stop_clicked();

private:
    Ui::DialogAvancedControl *ui;
};

#endif // DIALOGADVANCEDCONTROL_H
