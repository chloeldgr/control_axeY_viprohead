#ifndef SETHERCAT_H
#define SETHERCAT_H

#include <QObject>
#include <QDebug>
#include <QProcess>
#include <QTimer>
///
/// \brief The sEtherCAT class
/// \brief Classe contenant les fonctions permettant d'ouvrir et fermer la communication avec le buxs EtherCAT
///
class sEtherCAT : public QObject
{
    Q_OBJECT
public :
    explicit sEtherCAT(QObject *parent = 0);
    ~sEtherCAT(void);
    // accesseurs;
    bool getOpenStatus(void);
    bool getECMasterStatus(void);
    // modifieurs;
    void setOpenStatus(bool new_status);
public slots:
    void            open(void);
    void            close(void);
    void            refreshStatus(void);
    void            printOutput(void);
signals:
    void            IsOpened(void);
    void            IsClosed(void);
    void            ECMasterStatusChanged(void);
protected :
    bool    bIsOpen;
    bool    bECMasterIsRunning;
    QProcess        *p_Process;
    QTimer          *pTimerRefresh;
    QByteArray      result;

};
#endif // SETHERCAT_H
