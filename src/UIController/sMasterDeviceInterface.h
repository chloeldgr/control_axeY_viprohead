#ifndef SMASTERDEVICEINTERFACE_H
#define SMASTERDEVICEINTERFACE_H

#include <QDialog>
#include <sMasterRobot.h>

namespace Ui {
class MasterDeviceInterface;
}

class MasterDeviceInterface : public QDialog
{
    Q_OBJECT

public:
    explicit MasterDeviceInterface(QWidget *parent = 0);
    ~MasterDeviceInterface();
    sMasterRobot *pMasterRobot;
public slots:
    void            startTeleop(void);
    void            stopTeleop(void);
signals:
    void        teleopStarted(void);
    void        teleopStopped(void);
private:
    Ui::MasterDeviceInterface *ui;

};

#endif // SMASTERDEVICEINTERFACE_H
