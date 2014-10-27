#include "hubobalanceviewer.h"
#include <HpMotionMath.h>
#include "hubobalancemanage.h"

static HuboMotionData *referMotion;

HuboBalanceViewer::HuboBalanceViewer(QWidget *parent)
{
	HuboBalanceManage *manager = new HuboBalanceManage();
	manager->viewer = this;
	manager->show();
}

void HuboBalanceViewer::setCmaMotion(
		double kl, double kh,
		double weightTrack, double weightTrackAnkle, double weightTrackUpper,
		int frameRate
)
{
	if (referMotion == NULL)
	{
		std::cout << "Reference motion is empty. Please set a reference motion.(using setReferMotion())" << std::endl;
		return;
	}

	hubo->initController();
	hubo->huboVpBody->pHuboMotion->init();
	hubo->huboVpBody->pHuboMotion->setMotionSize(referMotion->getMotionSize());
	hubo->huboVpBody->pHuboMotion->setFrameRate(referMotion->getFrameRate());

	// set hybrid dynamics with floating base
	hubo->huboVpBody->initHybridDynamics(true);

	// loop for entire time
	int totalStep = (int)
		(
		referMotion->getFrameTime()
		* referMotion->getMotionSize()
		/hubo->timestep
		);
	double time;
	double framestep = 0;
	double frameTime = referMotion->getFrameTime();
	if(frameRate != 0 )
	{
		frameTime = 1.0/frameRate;
		int motionSize = (referMotion->getFrameTime() * referMotion->getMotionSize()/frameTime);
		hubo->huboVpBody->pHuboMotion->setMotionSize(motionSize);
	}

	hubo->huboVpBody->setInitialHuboHipFromMotion(referMotion);
	hubo->huboVpBody->setInitialHuboAngleFromMotion(referMotion);
	hubo->huboVpBody->setInitialHuboAngleRateFromMotion(referMotion);

	for (int i = 0; i < totalStep; i++)
	{
		time = i * hubo->timestep;
		framestep += hubo->timestep;

		hubo->balancing(
			referMotion, time,
			kl, kh,
			weightTrack, weightTrackAnkle, weightTrackUpper
		);

		hubo->huboVpBody->solveHybridDynamics();

		// go one time step
		hubo->stepAheadWithPenaltyForces();

		// set pose
		if (framestep >= frameTime)
		{
			framestep -= frameTime;
			hubo->huboVpBody->applyAllJointValueVptoHubo();
			if (hubo->huboVpBody->pHuboMotion->canGoOneFrame())
				hubo->huboVpBody->pHuboMotion->setCurrentFrame(hubo->huboVpBody->pHuboMotion->getCurrentFrame() + 1);
		}


	}
	adjustHuboMotionToViewer();
	hubo->huboVpBody->pHuboMotion->setCurrentFrame(0);
}

void HuboBalanceViewer::setReferMotion(HuboMotionData *refer)
{
	referMotion = refer;
	/*
	HuboCmaManage *huboTrMan = new HuboCmaManage;
	huboTrMan->initManager(this);
	huboTrMan->move(200+width()+20, 200);
	huboTrMan->setWindowTitle(QString("Cma Dialog"));
	huboTrMan->show();
	//*/
}
