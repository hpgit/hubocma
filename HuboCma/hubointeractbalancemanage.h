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

	void doingInOneStep(double time);

	double kl, kh, trackWeight, trackUpperWeight, linWeight, angWeight, torqueWeight;

private slots:
	void timer();

	void on_playBtn_clicked();

	void on_resetBtn_clicked();

	void on_pushBtn_clicked();

	void on_stepBtn_clicked();


	void on_hSlider_1_valueChanged(int value);
	void on_hSlider_2_valueChanged(int value);
	void on_hSlider_3_valueChanged(int value);
	void on_hSlider_4_valueChanged(int value);
	void on_hSlider_5_valueChanged(int value);
	void on_hSlider_6_valueChanged(int value);
	void on_hSlider_7_valueChanged(int value);

	void on_nStepBtn_clicked();

private:
	Ui::HuboInteractBalanceManage *ui;

	double pushTime;
	Eigen::VectorXd pushForce;
};

#endif // HUBOINTERACTBALANCEMANAGE_H
