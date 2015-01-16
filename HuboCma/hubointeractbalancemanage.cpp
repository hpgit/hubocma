#include "hubointeractbalancemanage.h"
#include "ui_hubointeractbalancemanage.h"

#include <QTimer>
#include "HpMotionMath.h"
#include <float.h>
#include <QLabel>

HuboInteractBalanceManage::HuboInteractBalanceManage(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::HuboInteractBalanceManage)
{
	ui->setupUi(this);

	simulTime = 0;
	simulTimeStep = 0.001;
	renderTimeStep = 0.033333;

	pushTime = 0;
	ui->pushForceXText->setPlainText(QString("30"));
	ui->pushForceYText->setPlainText(QString("0"));
	ui->pushForceZText->setPlainText(QString("0"));
	ui->nStepText->setPlainText(QString("10"));


	kl=1;
	kh=1;
	trackWeight=1;
	trackUpperWeight=1;
	linWeight=1;
	angWeight=1;
	torqueWeight=1;
}

HuboInteractBalanceManage::~HuboInteractBalanceManage()
{
	delete ui;
}

void HuboInteractBalanceManage::initTimer()
{
	simulTimeStep = viewer->glWidget->cont->getTimeStep();

	playTimer = new QTimer(this);
	playTimer->setInterval((int)(1000*renderTimeStep));
	connect(playTimer, SIGNAL(timeout()), this, SLOT(timer()));

	viewer->playing = 0;
	pushForce.resize(viewer->hubo->huboVpBody->bodies.size()*3);
	pushForce.setZero();

	//viewer->hubo->huboVpBody->setInitialHuboHipFromMotion(viewer->refer);
	//viewer->hubo->huboVpBody->setInitialHuboAngleFromMotion(viewer->refer);
	//viewer->hubo->huboVpBody->setInitialHuboAngleRateFromMotion(viewer->refer);
}

void HuboInteractBalanceManage::reset()
{
	if (playTimer->isActive())
		playTimer->stop();
	viewer->hubo->initController();

	simulTime = 0;
	simulTimeStep = viewer->glWidget->cont->getTimeStep();
	renderTimeStep = 0.033333;

	pushTime = 0;

	viewer->hubo->huboVpBody->setInitialHuboHipFromMotion(viewer->refer);
	viewer->hubo->huboVpBody->setInitialHuboAngleFromMotion(viewer->refer);
	viewer->hubo->huboVpBody->setInitialHuboAngleRateFromMotion(viewer->refer);
	viewer->hubo->huboVpBody->applyAllJointValueVptoHubo();
	viewer->glWidget->updateGL();
}

void HuboInteractBalanceManage::toggleTimerState()
{
	if(simulTime <DBL_EPSILON)
		reset();
	if (playTimer->isActive())
		playTimer->stop();
	else
		playTimer->start();
}

void HuboInteractBalanceManage::on_playBtn_clicked()
{
	toggleTimerState();
}

void HuboInteractBalanceManage::on_pushBtn_clicked()
{
	pushForce.resize(viewer->hubo->huboVpBody->bodies.size()*3);
	pushForce.setZero();
	pushForce(HuboVPBody::eTorso*3+0) = ui->pushForceXText->toPlainText().toDouble();
	pushForce(HuboVPBody::eTorso*3+1) = ui->pushForceYText->toPlainText().toDouble();
	pushForce(HuboVPBody::eTorso*3+2) = ui->pushForceZText->toPlainText().toDouble();
	pushTime += 0.2;
	Eigen::Vector3d force(
				ui->pushForceXText->toPlainText().toDouble(),
				ui->pushForceYText->toPlainText().toDouble(),
				ui->pushForceZText->toPlainText().toDouble());
	Eigen::Vector3d pos = Vec3Tovector(viewer->hubo->huboVpBody->Torso->GetFrame().GetPosition());
	viewer->glWidget->forceDrawOn(force, pos);
	viewer->glWidget->updateGL();
}

void HuboInteractBalanceManage::on_resetBtn_clicked()
{
	reset();
}


void HuboInteractBalanceManage::doingInOneStep(double time)
{
	//Eigen::VectorXd dofTorque;
	//viewer->hubo->balance(viewer->refer, time, 0.1, 1, 1, 1);
	//viewer->hubo->motionPdTracking(dofTorque, viewer->refer, time);
	//viewer->hubo->huboVpBody->applyAllJointTorque(dofTorque);
	viewer->hubo->balanceQp(viewer->refer,time,
							kl, kh,
							trackWeight, trackUpperWeight,
							linWeight, angWeight,torqueWeight
							);
}

void HuboInteractBalanceManage::timer()
{
	double totalTime = viewer->refer->getTotalTime();
	for(int i=0; i<renderTimeStep/simulTimeStep; i++)
	{
		double time = fmod(simulTime, totalTime);

		doingInOneStep(time);

		if(pushTime > DBL_EPSILON)
		{
			viewer->hubo->huboVpBody->applyAddAllBodyForce(pushForce);
			pushTime -= simulTimeStep;
		}
		else
			viewer->glWidget->forceDrawOff();
		viewer->hubo->stepAheadWithPenaltyForces();
		simulTime += simulTimeStep;
	}
	//simulTime += renderTimeStep;

	viewer->hubo->huboVpBody->applyAllJointValueVptoHubo();
	viewer->glWidget->updateGL();
}

void HuboInteractBalanceManage::on_stepBtn_clicked()
{
	Eigen::VectorXd dofTorque;
	double totalTime = viewer->refer->getTotalTime();
	double time = fmod(simulTime, totalTime);

	doingInOneStep(time);

	if(pushTime > DBL_EPSILON)
	{
		viewer->hubo->huboVpBody->applyAddAllBodyForce(pushForce);
		pushTime -= simulTimeStep;
	}
	else
		viewer->glWidget->forceDrawOff();
	viewer->hubo->stepAheadWithPenaltyForces();

	//if(simulTime > DBL_EPSILON)
	//{
	//	dJ_calc = (J-Jold)/simulTimeStep;
	//	std::cout << "dJ_calc:" << std::endl;
	//	std::cout << dJ_calc <<std::endl;
	//	std::cout << "dJ:" << std::endl;
	//	std::cout << dJ <<std::endl;
	//	std::cout << "diff: " <<std::endl;
	//	std::cout << (dJ-dJ_calc) << std::endl;
	//}
	//Jold = J;
	simulTime += simulTimeStep;

	viewer->hubo->huboVpBody->applyAllJointValueVptoHubo();
	viewer->glWidget->updateGL();
}


void HuboInteractBalanceManage::on_hSlider_1_valueChanged(int value)
{
	kl = std::pow(10,(double)value /100.0);
	if(value == -300)
		kl = 0;
	ui->label_2->setText(QString::number(kl,'g',3));
}
void HuboInteractBalanceManage::on_hSlider_2_valueChanged(int value)
{
	kh = std::pow(10,(double)value /100.0);
	if(value == -300)
		kh = 0;
	ui->label_4->setText(QString::number(kh,'g',3));
}
void HuboInteractBalanceManage::on_hSlider_3_valueChanged(int value)
{
	trackWeight    = std::pow(10,(double)value /100.0);
	if(value == -300)
		trackWeight = 0;
	ui->label_6->setText(QString::number(trackWeight,'g',3));
}
void HuboInteractBalanceManage::on_hSlider_4_valueChanged(int value)
{
	trackUpperWeight= std::pow(10,(double)value /100.0);
	if(value == -300)
		trackUpperWeight = 0;
	ui->label_8->setText(QString::number(trackUpperWeight,'g',3));
}
void HuboInteractBalanceManage::on_hSlider_5_valueChanged(int value)
{
	linWeight= std::pow(10,(double)value /100.0);
	if(value == -300)
		linWeight = 0;
	ui->label_10->setText(QString::number(linWeight,'g',3));
}
void HuboInteractBalanceManage::on_hSlider_6_valueChanged(int value)
{
	angWeight = std::pow(10,(double)value /100.0);
	if(value == -300)
		angWeight = 0;
	ui->label_12->setText(QString::number(angWeight,'g',3));
}
void HuboInteractBalanceManage::on_hSlider_7_valueChanged(int value)
{
	torqueWeight= std::pow(10,(double)value /100.0);
	if(value == -300)
		torqueWeight = 0;
	ui->label_14->setText(QString::number(torqueWeight,'g',3));
}

void HuboInteractBalanceManage::on_nStepBtn_clicked()
{
	const double totalTime = viewer->refer->getTotalTime();
	const int nStep = ui->nStepText->toPlainText().toInt();
	for(int i=0; i<nStep; i++)
	{
		double time = fmod(simulTime, totalTime);

		doingInOneStep(time);

		if(pushTime > DBL_EPSILON)
		{
			viewer->hubo->huboVpBody->applyAddAllBodyForce(pushForce);
			pushTime -= simulTimeStep;
		}
		else
			viewer->glWidget->forceDrawOff();
		viewer->hubo->stepAheadWithPenaltyForces();
		simulTime += simulTimeStep;
	}
	//simulTime += renderTimeStep;

	viewer->hubo->huboVpBody->applyAllJointValueVptoHubo();
	viewer->glWidget->updateGL();
}
