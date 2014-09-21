#ifndef HUBOMAINCONTROLLER_H
#define HUBOMAINCONTROLLER_H

#include <QMainWindow>

namespace Ui {
class HuboMainController;
}

class HuboMainController : public QMainWindow
{
    Q_OBJECT

public:
    explicit HuboMainController(QWidget *parent = 0);
    ~HuboMainController();

private slots:
    //void on_loadRefBtn_clicked();

private:
    Ui::HuboMainController *ui;
};

#endif // HUBOMAINCONTROLLER_H
