#include "qtcontrollertask.h"
#include <QDebug>

QtControllerTask::QtControllerTask()
{
    qDebug() << "QtControllerTask()";
    cmd = 0;
}

void QtControllerTask::setTaskType(int new_cmd)
{
   cmd = new_cmd;
}

void QtControllerTask::run()
{
    // time consumer

    qDebug() << "Task start";
    int iNumber = 0;
    switch(cmd)
    {
        case :
        iNumber = 1;
        break;
    case 2:
        iNumber=200;
        break;
    default :
        iNumber = 999;
        break;
    }


    qDebug() << "Task done";
    emit Result(iNumber);
}

