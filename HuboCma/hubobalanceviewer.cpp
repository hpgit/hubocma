#include "hubobalanceviewer.h"
#include <HpMotionMath.h>

//static HuboVpController *huboCont;
static HuboMotionData *referMotion;

HuboBalanceViewer::HuboBalanceViewer(QWidget *parent)
{
}

void HuboBalanceViewer::setCmaMotion(int frameRate)
{
	if (referMotion == NULL)
	{
		std::cout << "Reference motion is empty. Please set a reference motion.(using setReferMotion())" << std::endl;
		return;
	}
	double Ks = hubo->ks;
	double Kd = hubo->kd;

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

	Eigen::VectorXd angleRefer, angleVelRefer, angleAccelRefer, desAccel;
	Eigen::VectorXd angle, angleVel, torques;

	Eigen::Vector3d hipPosRefer, hipVelRefer, hipAccelRefer, hipDesAccel;
	Eigen::Vector3d hipPos, hipVel;

	Eigen::Vector3d hipAngVelRefer, hipAngAccelRefer, hipDesAngAccel;
	Eigen::Vector3d hipAngVel;
	Eigen::Quaterniond hipOrienRefer;
	Eigen::Quaterniond hipOrien;

	hubo->huboVpBody->setInitialHuboHipFromMotion(referMotion);
	hubo->huboVpBody->setInitialHuboAngleFromMotion(referMotion);
	hubo->huboVpBody->setInitialHuboAngleRateFromMotion(referMotion);

	for (int i = 0; i < totalStep; i++)
	//for (int i = 0; i <= 350; i++)
	{
		time = i * hubo->timestep;
		framestep += hubo->timestep;

		/*
		// get desired acceleration for instance time
		hipPosRefer = referMotion->getHipJointGlobalPositionInTime(time);
		hipOrienRefer = referMotion->getHipJointGlobalOrientationInTime(time);
		hipVelRefer = referMotion->getHipJointVelInHuboMotionInTime(time);
		hipAngVelRefer = referMotion->getHipJointAngVelInHuboMotionInTime(time);
		hipAccelRefer = referMotion->getHipJointAccelInHuboMotionInTime(time);
		hipAngAccelRefer = referMotion->getHipJointAngAccelInHuboMotionInTime(time);

		hipPos = Vec3Tovector(hubo->huboVpBody->Hip->GetFrame().GetPosition());
		hipVel = Vec3Tovector(hubo->huboVpBody->Hip->GetLinVelocity(Vec3(0,0,0)));
		hipOrien = hubo->huboVpBody->getOrientation(hubo->huboVpBody->Hip);
		hipAngVel = Vec3Tovector(hubo->huboVpBody->Hip->GetAngVelocity());

		referMotion->getAllAngleInHuboMotionInTime(time, angleRefer);
		referMotion->getAllAngleRateInHuboMotionInTime(time, angleVelRefer);
		referMotion->getAllAngleAccelInHuboMotionInTime(time, angleAccelRefer);

		//TODO:
		//balancing


		hubo->huboVpBody->getAllAngle(angle);
		hubo->huboVpBody->getAllAngularVelocity(angleVel);

		// get desired accelaration
		hubo->getDesiredDofAccelForRoot(hipPosRefer, hipPos, hipVelRefer, hipVel, hipAccelRefer, hipDesAccel);
		hubo->getDesiredDofAngAccelForRoot(hipOrienRefer, hipOrien, hipAngVelRefer, hipAngVel, hipAngAccelRefer, hipDesAngAccel);

		hubo->getDesiredDofAccel(angleRefer, angle, angleVelRefer, angleVel, angleAccelRefer, desAccel);

		// set acceleration to joint
		hubo->huboVpBody->applyRootJointDofAccel(hipDesAccel, hipDesAngAccel);
		hubo->huboVpBody->applyAllJointDofAccel(desAccel);
		//*/

		std::cout << "balancing " << time << " : " << std::endl;
		hubo->balancing(referMotion, time);

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
