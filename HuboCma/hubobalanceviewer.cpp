#include "hubobalanceviewer.h"
#include <HpMotionMath.h>
#include "hubobalancemanage.h"

static HuboMotionData *referMotion;

HuboBalanceViewer::HuboBalanceViewer(QWidget *parent)
{
	HuboBalanceManage *manager = new HuboBalanceManage();
	manager->viewer = this;
	manager->move(200+width()+20, 200);
	manager->show();
}
void HuboBalanceViewer::setBalanceMotion(
	double kl, double kh,
	double weightTrack, double stepNum, double weightTrackUpper,
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
	hubo->huboVpBody->pHuboMotion->setFrameRate(referMotion->getFrameRate());

	// set hybrid dynamics with floating base
	hubo->huboVpBody->initHybridDynamics(false);

	// loop for entire time
	int totalStep = (int)
		(
		referMotion->getFrameTime()
		* (referMotion->getMotionSize()-1)
		/hubo->timestep
		);
	double time;
	double framestep = 0;
	double frameTime = referMotion->getFrameTime();
	if(frameRate != 0 )
	{
		frameTime = 1.0/frameRate;
	}

	hubo->huboVpBody->setInitialHuboHipFromMotion(referMotion);
	hubo->huboVpBody->setInitialHuboAngleFromMotion(referMotion);
	hubo->huboVpBody->setInitialHuboAngleRateFromMotion(referMotion);

	if(stepNum == 0)
		stepNum = totalStep;

	int motionSize = stepNum*hubo->timestep/frameTime;
	hubo->huboVpBody->pHuboMotion->setMotionSize(motionSize);

	int j = 0;
	//for (int i = 0; i < totalStep; i++)
	for (int i = 0; i < stepNum; i++)
	{
		//std::cout << j << "th : ";
		time = i * hubo->timestep;
		framestep += hubo->timestep;
		/*
		hubo->balancing(referMotion, time,
						1, 2, 100, 100, 1);
*/

		hubo->balance(
			referMotion, time,
			//referMotion, 0,
			kl, kh,
			weightTrack, weightTrackUpper
		);

		// go one time step
		hubo->stepAheadWithPenaltyForces();

		// set pose
		if (framestep >= frameTime)
		{
			j++;
			framestep -= frameTime;
			hubo->huboVpBody->applyAllJointValueVptoHubo();
			if (hubo->huboVpBody->pHuboMotion->canGoOneFrame())
				hubo->huboVpBody->pHuboMotion->setCurrentFrame(hubo->huboVpBody->pHuboMotion->getCurrentFrame() + 1);
		}
	}
	adjustHuboMotionToViewer();
	hubo->huboVpBody->pHuboMotion->setCurrentFrame(0);
	this->glWidget->repaint();
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
		* (referMotion->getMotionSize()-1)
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

	//TOOD:
	//for debug


	//for (int i = 0; i < totalStep; i++)
	for (int i = 0; i < 1000; i++)
	//for (int i = 0; i < 1; i++)
	{
		//std::cout << i << "th supCenter : ";
		time = i * hubo->timestep;
		framestep += hubo->timestep;

		hubo->balancing(
			referMotion, time,
			//referMotion, 0,
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
		if(i == -1)
		{
			Eigen::Vector3d velR, angVelR;
			Eigen::Quaterniond oriR;
			Eigen::Vector3d posR = referMotion->getHipJointGlobalPositionInTime(time);
			Eigen::Quaterniond referOri = referMotion->getHipJointGlobalOrientationInTime(time);
			std::cout << referOri.w() << " " << referOri.x() << " " << referOri.y() << " " << referOri.z() << std::endl;
			Eigen::Vector3d posV, velV, angVelV;
			Eigen::Quaterniond oriV;
			hubo->huboVpBody->getHuboHipState(posV, oriV, velV, angVelV);
			std::cout << oriV.w() << " " << oriV.x() << " " << oriV.y() << " " << oriV.z() <<std::endl;
		}


	}
	adjustHuboMotionToViewer();
	hubo->huboVpBody->pHuboMotion->setCurrentFrame(0);
	this->glWidget->repaint();
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
