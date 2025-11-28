#include "mainwindow.h"
#include <QApplication>



int main(int argc, char *argv[])
{
    int ret;
    QApplication a(argc, argv);
    MainWindow w;

    w.show();
    ret = a.exec();

    return ret;
}
