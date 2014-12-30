#ifndef HUBOMAINCONTROLLER_H
#define HUBOMAINCONTROLLER_H

#ifdef WIN32
#pragma warning(disable:4189)
#pragma warning(disable:4100)
#endif

#include <QMainWindow>
#include <HuboVpController.h>
//#include <HuboGearController.h>

namespace Ui {
class HuboMainController;
}

class HuboMainController : public QMainWindow
{
    Q_OBJECT

public:
    explicit HuboMainController(QWidget *parent = 0);
    ~HuboMainController();
	HuboVpController *huboRefer, *huboIk, *huboCma, *huboBalance, *huboInterBalance;

private slots:
    void on_loadReferBtn_clicked();

	void on_ikDlgBtn_clicked();

	void on_cmaDlgBtn_clicked();

	void on_balanceDlgBtn_clicked();

	void on_interBalanceDlgBtn_clicked();

private:
    Ui::HuboMainController *ui;
};

#endif // HUBOMAINCONTROLLER_H
