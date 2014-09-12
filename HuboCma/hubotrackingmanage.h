#ifndef HUBOTRACKINGMANAGE_H
#define HUBOTRACKINGMANAGE_H

#include <QDialog>

namespace Ui {
class HuboTrackingManage;
}

class HuboTrackingManage : public QDialog
{
    Q_OBJECT

public:
    explicit HuboTrackingManage(QWidget *parent = 0);
    ~HuboTrackingManage();

private slots:
    void on_simulBtn_clicked();

    void on_runCmaWithThis_clicked();

    void on_runCmaWithRes_clicked();

private:
    Ui::HuboTrackingManage *ui;
};

#endif // HUBOTRACKINGMANAGE_H
