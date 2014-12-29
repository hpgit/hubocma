#ifndef HUBOINTERACTBALANCEMANAGE_H
#define HUBOINTERACTBALANCEMANAGE_H

#include <QDialog>
#include <QTimer>
#include "hubointeractbalanceviewer.h"

namespace Ui {
class HuboInteractBalanceManage;
}

class HuboInteractBalanceManage : public QDialog
{
	Q_OBJECT

public:
	explicit HuboInteractBalanceManage(QWidget *parent = 0);
	~HuboInteractBalanceManage();

	HuboInteractBalanceViewer *viewer;
	QTimer *playTimer;
	double simulTime;
	double simulTimeStep;
	double renderTimeStep;

	void toggleTimerState();
	void reset();
	void initTimer();

private slots:
	void timer();

	void on_playBtn_clicked();

	void on_resetBtn_clicked();

	void on_pushBtn_clicked();

	void on_stepBtn_clicked();

private:
	Ui::HuboInteractBalanceManage *ui;

	double pushTime;
	Eigen::VectorXd pushForce;
};

#endif // HUBOINTERACTBALANCEMANAGE_H
