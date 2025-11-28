#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <sRTController.h>
#include <sEtherCAT.h>
#include <DialogInchWormActuator.h>
#include <dialogflatinchwormctrl.h>
#include <dialogSPIRITControl.h>
#include <DialogAdvancedControl.h>
#include <DialogWren.h>

class QAction;
class QActionGroup;
class QLabel;
class QMenu;
class QMessageBox;

namespace Ui {
class MainWindow;
class DialogInchWormActuator;
class DialogFlatInchwormCtrl;
class DialogWren;
class dialogSPIRITControl;
class DialogAvancedControl;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    sRTController           *pController;
    sEtherCAT               *pEtherCATMaster;
    DialogInchWormActuator  *pDialogInchworm;
    DialogFlatInchwormCtrl  *pDialogFlatInchworm;
    DialogSPIRITControl     *pDialogSPIRITControl;
    DialogAvancedControl    *pDialogAdvancedControl;
    DialogWren              *pDialogWren;
    int analog_value[6];
private slots:
    void about();
    void ChangeSlaveControllerMode(mode_type);
    void UpdateAnalogInput(double *);
    void UpdateDisplay(void);
    void setDigitalOutput(void);
    void setDigOutEV(void);
    void ChangeECMasterStatus(void);
    void CtrlWrenPressure(int,double);
    void CtrlWrenOutput(int, bool);
    void CtrlGrasping(bool);
signals:
    void changeDigitalOuput(int);
private:
    Ui::MainWindow *ui;

    void createActions();
    void createMenus();

    QMenu *mainAppMenu;
    QMenu *EtherCATMenu;
    QMenu *SlaveRTControllerMenu;
    QMenu *RobotControlMenu;
    QMenu *helpMenu;
    QActionGroup *alignmentGroup;
    QAction *openIdentificationWindow;

    QAction *aboutAct;
    QAction *openEtherCATAct;
    QAction *closeEtherCATAct;
    QAction *startRTControllerAct;
    QAction *stopRTControllerAct;
    QAction *inchwormAct;

    QAction *flatinchwormAct;
    QAction *Open_dialogSPIRITControl;
    QAction *Open_DialogAdvancedControl;
    QAction *Open_DialogWren;
    QAction *exitAct;

};

#endif // MAINWINDOW_H
