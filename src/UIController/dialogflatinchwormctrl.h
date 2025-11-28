#ifndef DIALOGFLATINCHWORMCTRL_H
#define DIALOGFLATINCHWORMCTRL_H

#include <QDialog>
#include <QDebug>
namespace Ui {
class DialogFlatInchwormCtrl;
}

class DialogFlatInchwormCtrl : public QDialog
{
    Q_OBJECT

public:
    explicit DialogFlatInchwormCtrl(QWidget *parent = 0);
    ~DialogFlatInchwormCtrl();

private:
    Ui::DialogFlatInchwormCtrl *ui;

private slots:
    void onBtnStart(void);

signals:
    void start(int, double, bool);
    void stop(void);

};

#endif // DIALOGFLATINCHWORMCTRL_H
