#ifndef QTCONTROLLERTASK_H
#define QTCONTROLLERTASK_H

#include <QRunnable>
#include <QObject>
#include <QRunnable>


class QtControllerTask : public QObject, public QRunnable
{
    Q_OBJECT
public:
    QtControllerTask();
    setTaskType(int cmd);
signals:
    void Result(int Number);
protected:
    int     cmd;
    void run();

};

#endif // QTCONTROLLERTASK_H
