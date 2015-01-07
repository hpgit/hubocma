#include "hubointeractbalancemanage.h"
#include "ui_hubointeractbalancemanage.h"

#include <QTimer>
#include "HpMotionMath.h"
#include <float.h>

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

void HuboInteractBalanceManage::timer()
{
	Eigen::VectorXd dofTorque, b;
	Eigen::MatrixXd M;
	double totalTime = viewer->refer->getTotalTime();
	for(int i=0; i<renderTimeStep/simulTimeStep; i++)
	{
		double time = fmod(simulTime, totalTime);
		viewer->hubo->motionPdTracking(dofTorque, viewer->refer, fmod(simulTime, totalTime));
		viewer->hubo->huboVpBody->applyAllJointTorque(dofTorque);
		//viewer->hubo->balance(viewer->refer, time, 0.1, 1, 1, 1);
		if(pushTime > DBL_EPSILON)
		{
			viewer->hubo->huboVpBody->applyAddAllBodyForce(pushForce);
			pushTime -= simulTimeStep;
		}
		else
			viewer->glWidget->forceDrawOff();
		//viewer->hubo->huboVpBody->getEquationsOfMotion(viewer->hubo->world, M, b);
		viewer->hubo->stepAheadWithPenaltyForces();
	}
	simulTime += renderTimeStep;

	viewer->hubo->huboVpBody->applyAllJointValueVptoHubo();
	viewer->glWidget->updateGL();
}

static Eigen::MatrixXd Jold;

void HuboInteractBalanceManage::on_stepBtn_clicked()
{
	Eigen::VectorXd dofTorque, b;
	Eigen::MatrixXd M;
	double totalTime = viewer->refer->getTotalTime();
	double time = fmod(simulTime, totalTime);
	//viewer->hubo->balance(viewer->refer, time, 0.1, 1, 1, 1);
	viewer->hubo->motionPdTracking(dofTorque, viewer->refer, fmod(simulTime, totalTime));
	viewer->hubo->huboVpBody->applyAllJointTorque(dofTorque);
	if(pushTime > DBL_EPSILON)
	{
		viewer->hubo->huboVpBody->applyAddAllBodyForce(pushForce);
		pushTime -= simulTimeStep;
	}
	else
		viewer->glWidget->forceDrawOff();
	//viewer->hubo->huboVpBody->getEquationsOfMotion(viewer->hubo->world, M, b);
	//Eigen::MatrixXd J, dJ, dJ_calc;
	//viewer->hubo->huboVpBody->getJacobian(J, 1);
	//viewer->hubo->huboVpBody->getDifferentialJacobian(dJ, 1);
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
	//std::cout << "M: " <<std::endl;
	//std::cout << M <<std::endl;
	//std::cout << "b: " <<std::endl;
	//std::cout << b.transpose() <<std::endl;
}
