#include "HuboVpController.h"
#include "HpMotionMath.h"

HuboVpController::~HuboVpController()
{
	if(!huboVpBody)
		delete huboVpBody;
}

void HuboVpController::init()
{
	importHuboSkeleton("../../dat/RobotData/RobotData.dat");
	initController();
}

void HuboVpController::importHuboSkeleton(char* filename)
{
	if(huboMotion)
		delete huboMotion;
	huboMotion = new HuboMotionData;
	huboMotion->importSkeleton(filename);
}

void HuboVpController::initController(void)
{
	int frame = huboMotion->getCurrentFrame();
	if(world)
		delete world;
	world = new vpWorld;
	world->SetGravity(Vec3(0.0, -9.8, 0.0));

	if(ground)
		delete ground;
	ground = new vpBody;
	ground->SetGround();
	
	if(groundGeom)
		delete groundGeom;
	//groundGeom = new vpPlane(Vec3(0, 1, 0));
	groundGeom = new vpBox(Vec3(100.0, 5.0, 100.0));
	//ground->AddGeometry(groundGeom);
	ground->AddGeometry(groundGeom, Vec3(0,-2.5,0));
	world->AddBody(ground);

	if(huboVpBody)
		delete huboVpBody;
	huboVpBody = new HuboVPBody;
	huboMotion->resetMotion();
	Vector3d y1(0,1,0);
	huboMotion->jointMap["Hip"]->setTranslation(frame, y1);
	huboMotion->makeGroundContact();
	huboVpBody->create(world, huboMotion);
	huboVpBody->grfDs = grfDs;
	huboVpBody->grfKs = grfKs;
	huboVpBody->ignoreVpHuboBodyCollision(world);
	huboVpBody->ignoreVpGroundBodyCollision(world, ground);

	huboVpBody->Hip->SetFrame(
		Vec3(huboMotion->jointMap["Hip"]->getGlobalPosition(frame).data())
		);
	huboVpBody->Hip->SetFrame(
		Vec3(huboMotion->jointMap["Hip"]->getGlobalBoundingBoxPosition(frame).data())
		);

	
#ifndef __APPLE__
	world->SetNumThreads(omp_get_num_threads()-1);
#endif
	world->SetIntegrator(VP::IMPLICIT_EULER_FAST);
	world->SetTimeStep(timestep);

	world->Initialize();
	world->BackupState();
}

void HuboVpController::setTimeStep(double _timestep)
{
	timestep = _timestep;
}

double HuboVpController::getTimeStep()
{
	return timestep;
}
void HuboVpController::stepAheadWithPenaltyForces()
{
	cpBeforeOneStep = huboVpBody->getCOPposition(world, ground);
	huboVpBody->stepAhead(world, ground);
	//cp = huboVpBody->getCOPposition(world, ground);
}

void HuboVpController::applyPdControlTorque(
	Eigen::VectorXd &desireAngles, 
	Eigen::VectorXd &desireAngularVelocities
	)
{
	Eigen::VectorXd torque;
	Eigen::VectorXd currentAngles;
	Eigen::VectorXd currentAngularVelocities;

	huboVpBody->getAllAngle(currentAngles);
	torque = ks * (desireAngles - currentAngles)
		+ kd * (desireAngularVelocities - currentAngularVelocities);
}

void HuboVpController::getDesiredDofAngAccelForRoot(
	Eigen::Quaterniond &orienRefer,
	Eigen::Quaterniond &orienVp,
	Eigen::Vector3d &angleRateRefer,
	Eigen::Vector3d &angleRateVp,
	Eigen::Vector3d &angleAccelRefer,
	Eigen::Vector3d &angacc
	)
{
//	ddth_des[i] 
//	= Kt*(mm.logSO3(np.dot(th[i].transpose(), th_r[i]))) 
//	+ Dt*(dth_r[i] - dth[i]) 
//	+ ddth_r[i];
	angacc = ks*diffQuat(orienRefer, orienVp)
		+ kd*(angleRateRefer - angleRateVp)
		+ angleAccelRefer;
}

void HuboVpController::getDesiredDofAccelForRoot(
	Eigen::Vector3d &posRefer,
	Eigen::Vector3d &posVp,
	Eigen::Vector3d &posRateRefer,
	Eigen::Vector3d &posRateVp,
	Eigen::Vector3d &posAccelRefer,
	Eigen::Vector3d &acc
	)
{
	acc = ks*(posRefer - posVp)
		+ kd*(posRateRefer - posRateVp)
		+ posAccelRefer;
}

void HuboVpController::getDesiredDofAccel(
	Eigen::VectorXd &angleRefer,
	Eigen::VectorXd &angleVp,
	Eigen::VectorXd &angleRateRefer,
	Eigen::VectorXd &angleRateVp,
	Eigen::VectorXd &angleAccelRefer,
	Eigen::VectorXd &acc
	)
{
	acc.resize(26);
	//for (int i = 0; i < acc.size(); i++)
		acc = ks*(angleRefer - angleVp) 
			+ kd*(angleRateRefer - angleRateVp) 
			+ angleAccelRefer;

}

void HuboVpController::motionPdTrackingThread(HuboVpController *cont, HuboMotionData *referMotion)
{
	cont->huboVpBody->pHuboMotion->setMotionSize(referMotion->getMotionSize());
	cont->huboVpBody->pHuboMotion->setFrameRate(referMotion->getFrameRate());

	// set hybrid dynamics with floating base
	cont->huboVpBody->initHybridDynamics(true);

	// loop for entire time
	int totalStep = (int)
		(
		referMotion->getFrameTime() 
		* referMotion->getMotionSize()
		/cont->timestep
		);
	double time;
	double framestep = 0;
	double frameTime = referMotion->getFrameTime();

	Eigen::VectorXd angleRefer, angleVelRefer, angleAccelRefer, desAccel;
	Eigen::VectorXd angle, angleVel;

	Eigen::Vector3d hipPosRefer, hipVelRefer, hipAccelRefer, hipDesAccel;
	Eigen::Vector3d hipPos, hipVel;

	Eigen::Vector3d hipAngVelRefer, hipAngAccelRefer, hipDesAngAccel;
	Eigen::Vector3d hipAngVel;
	Eigen::Quaterniond hipOrienRefer;
	Eigen::Quaterniond hipOrien;

	for (int i = 0; i <= totalStep; i++)
	{
		time = i * cont->timestep;
		framestep += cont->timestep;

		// get desired acceleration for instance time
		hipPosRefer = referMotion->getHipJointGlobalPositionInTime(time);
		hipOrienRefer = referMotion->getHipJointGlobalOrientationInTime(time);
		hipVelRefer = referMotion->getHipJointVelInHuboMotionInTime(time);
		hipAngVelRefer = referMotion->getHipJointAngVelInHuboMotionInTime(time);
		hipAccelRefer = referMotion->getHipJointAccelInHuboMotionInTime(time);
		hipAngAccelRefer = referMotion->getHipJointAngAccelInHuboMotionInTime(time);

		hipPos = Vec3Tovector(cont->huboVpBody->Hip->GetFrame().GetPosition());
		hipVel = Vec3Tovector(cont->huboVpBody->Hip->GetLinVelocity(Vec3(0,0,0)));
		hipOrien = cont->huboVpBody->getOrientation(cont->huboVpBody->Hip);
		hipAngVel = Vec3Tovector(cont->huboVpBody->Hip->GetAngVelocity());

		referMotion->getAllAngleInHuboMotionInTime(time, angleRefer);
		referMotion->getAllAngleRateInHuboMotionInTime(time, angleVelRefer);
		referMotion->getAllAngleAccelInHuboMotionInTime(time, angleAccelRefer);

		cont->huboVpBody->getAllAngle(angle);
		cont->huboVpBody->getAllAngularVelocity(angleVel);

		cont->getDesiredDofAccelForRoot(hipPosRefer, hipPos, hipVelRefer, hipVel, hipAccelRefer, hipDesAccel);
		cont->getDesiredDofAngAccelForRoot(hipOrienRefer, hipOrien, hipAngVelRefer, hipAngVel, hipAngAccelRefer, hipDesAngAccel);

		cont->getDesiredDofAccel(angleRefer, angle, angleVelRefer, angleVel, angleAccelRefer, desAccel);

		// set acceleration to joint
		cont->huboVpBody->applyRootJointDofAccel(hipDesAccel, hipDesAngAccel);
		cont->huboVpBody->applyAllJointDofAccel(desAccel);
		
		//std::cout << desAccel.transpose() << std::endl;

		cont->huboVpBody->solveHybridDynamics();
	
		// go one time step
		cont->huboVpBody->stepAhead(cont->world, cont->ground);
		
		// set pose
		if (framestep >= frameTime)
		{
			framestep -= frameTime;
			cont->huboVpBody->applyAllJointValueVptoHubo();
			if (cont->huboVpBody->pHuboMotion->canGoOneFrame())
				cont->huboVpBody->pHuboMotion->setCurrentFrame(cont->huboVpBody->pHuboMotion->getCurrentFrame() + 1);
		}
	}

	//cont->huboVpBody->pHuboMotion->setCurrentFrame(0);

}

void HuboVpController::balancing(
		HuboMotionData *referMotion, double time,
		double kl, double kh,
		double weightTrack, double weightTrackAnkle, double weightTrackUpper
)
{
	double dl=2*std::sqrt(kl), dh=2*std::sqrt(kh);

	//[Macchietto 2009]
	Eigen::MatrixXd M, dM, J, dJ, Jsup, dJsup;
	huboVpBody->getJacobian(J);
	huboVpBody->getLinkMatrix(M);
	huboVpBody->getDifferentialJacobian(dJ);
	huboVpBody->getDifferentialLinkMatrix(dM);

	Eigen::VectorXd angles, angVels, angAccels;
	Eigen::VectorXd dofVels;
	huboVpBody->getAllAngle(angles);
	huboVpBody->getAllAngularVelocity(angVels);
	dofVels.resize(32);

	Vec3 rootVel, rootAngVel;
	rootVel = huboVpBody->Hip->GetLinVelocity(Vec3(0,0,0));
	rootAngVel = huboVpBody->Hip->GetAngVelocity();

	dofVels.segment(0, 3) = Vec3Tovector(rootVel);
	dofVels.segment(3, 3) = Vec3Tovector(rootAngVel);
	dofVels.tail(26) = angVels;
	//std::cout << "dofVels: " << dofVels.transpose() << std::endl;

	Eigen::MatrixXd RS = M*J;
	Eigen::MatrixXd R = RS.block(0, 0, 3, 32);
	Eigen::MatrixXd S = RS.block(3, 0, 3, 32);

	Eigen::VectorXd rs = (dM*J+M*dJ)*dofVels;
	//std::cout << "rs: " << rs.transpose() << std::endl;

	Eigen::Vector3d rbias = rs.head(3);
	Eigen::Vector3d sbias = rs.tail(3);

	//calculate LdotDes, HdotDes
	Eigen::Vector3d LdotDes, HdotDes;
	
	Eigen::Vector3d supCenter = huboVpBody->getSupportRegionCenter();
	Eigen::Vector3d comPlane = huboVpBody->getCOMposition();
	comPlane.y() = 0;

	//LdotDes
	LdotDes = huboVpBody->mass *(
		kl * (supCenter - comPlane)
		- dl * huboVpBody->getCOMvelocity()
			);
	LdotDes.y() = 0;
	//std::cout << "kl term: " << (supCenter-comPlane).transpose()
	//		  << "com velocity : " << huboVpBody->getCOMvelocity().transpose()
	//		  << ",  LdotDes: " << LdotDes.transpose()
	//		  << std::endl;
	

	//HdotDes
//        # angular momentum
//        CP_ref = footCenter
//        bodyIDs, contactPositions, contactPositionLocals, contactForces = vpWorld.calcPenaltyForce(bodyIDsToCheck, mus, Ks, Ds)
//        CP = yrp.getCP(contactPositions, contactForces)
//        if CP_old[0]==None or CP==None:
//            dCP = None
//        else:
//            dCP = (CP - CP_old[0])/(1/30.)
//        CP_old[0] = CP            
//
//        if CP!=None and dCP!=None:
//            ddCP_des = Kh*(CP_ref - CP) - Dh*(dCP)
//            CP_des = CP + dCP*(1/30.) + .5*ddCP_des*((1/30.)**2)
//            dH_des = np.cross((CP_des - CM), (dL_des_plane + totalMass*mm.s2v(wcfg.gravity)))
//        else:
//            dH_des = None

	//cpBeforeOneStep is calculated in stepAheadWithPenaltyForces()
	Eigen::Vector3d cp = huboVpBody->getCOPposition(world, ground);
	Eigen::Vector3d cpOld, pdes, dp, ddpdes;
	int bCalCP = 1;

	//std::cout << "cp: " <<cp.transpose() << std::endl;

	//TODO : when cp.y() < 0
	if (cp.y() < 0)
	{
		HdotDes = Vector3d(0, 0, 0);
		bCalCP = 0;
	}
	else
	{
		cpOld = this->cpBeforeOneStep;
		Eigen::Vector3d pdes, dp, ddpdes;
		Eigen::Vector3d refSupCenter =
				huboVpBody->pHuboMotion->getFootCenterInTime(time);

		dp = cp - cpOld;

		ddpdes = kh*(refSupCenter - cp) - dh*dp;
		pdes = cp + dp*timestep + 0.5 * ddpdes*timestep*timestep;
		HdotDes = (pdes - comPlane).cross(LdotDes - huboVpBody->mass * Vector3d(0, -9.8, 0));
	}
	//std::cout << "HdotDes: " << HdotDes.transpose() << std::endl;

	// tracking term
	Eigen::VectorXd desDofAccel;
	desDofAccel.resize(32);
	{
		Eigen::VectorXd angleRefer, angleVelRefer, angleAccelRefer;
		Eigen::VectorXd desAccel;

		Eigen::Vector3d hipPosRefer, hipVelRefer, hipAccelRefer, hipDesAccel;
		Eigen::Vector3d hipPos, hipVel;
		Eigen::Vector3d hipAngVelRefer, hipAngAccelRefer, hipDesAngAccel;
		Eigen::Vector3d hipAngVel;
		Eigen::Quaterniond hipOrienRefer;
		Eigen::Quaterniond hipOrien;
		 
		// get desired acceleration for instance time
		hipPosRefer = referMotion->getHipJointGlobalPositionInTime(time);
		hipOrienRefer = referMotion->getHipJointGlobalOrientationInTime(time);
		hipVelRefer = referMotion->getHipJointVelInHuboMotionInTime(time);
		hipAngVelRefer = referMotion->getHipJointAngVelInHuboMotionInTime(time);
		hipAccelRefer = referMotion->getHipJointAccelInHuboMotionInTime(time);
		hipAngAccelRefer = referMotion->getHipJointAngAccelInHuboMotionInTime(time);

		hipPos = Vec3Tovector(huboVpBody->Hip->GetFrame().GetPosition());
		hipVel = Vec3Tovector(huboVpBody->Hip->GetLinVelocity(Vec3(0,0,0)));
		hipOrien = huboVpBody->getOrientation(huboVpBody->Hip);
		hipAngVel = Vec3Tovector(huboVpBody->Hip->GetAngVelocity());

		referMotion->getAllAngleInHuboMotionInTime(time, angleRefer);
		referMotion->getAllAngleRateInHuboMotionInTime(time, angleVelRefer);
		referMotion->getAllAngleAccelInHuboMotionInTime(time, angleAccelRefer);

		getDesiredDofAccelForRoot(hipPosRefer, hipPos, hipVelRefer, hipVel, hipAccelRefer, hipDesAccel);
		getDesiredDofAngAccelForRoot(hipOrienRefer, hipOrien, hipAngVelRefer, hipAngVel, hipAngAccelRefer, hipDesAngAccel);
		getDesiredDofAccel(angleRefer, angles, angleVelRefer, angVels, angleAccelRefer, desAccel);

		desDofAccel.head(3) = hipDesAccel;
		desDofAccel.segment(3,3) = hipDesAngAccel;
		desDofAccel.tail(26) = desAccel;

		//std::cout << "desAccel :" << desAccel.transpose() << std::endl;
	}

	// constraint
	// calculate Jsup and dJsup
	huboVpBody->getFootSupJacobian(J, Jsup);
	huboVpBody->getDifferentialFootSupJacobian(dJ, dJsup);

	// linear equation
	Eigen::MatrixXd A;
	Eigen::VectorXd b;

	//A.resize(32, 32);
	A.resize(32 + Jsup.rows(), 32 + Jsup.rows());
	A.setZero();
	A.block(0, 0, 32, 32).setIdentity();
	A.block(0, 0, 32, 32) *= 2*weightTrack;

	A(6+HuboVPBody::eRAR, 6+HuboVPBody::eRAR) = 2*weightTrackAnkle;
	A(6+HuboVPBody::eRAP, 6+HuboVPBody::eRAP) = 2*weightTrackAnkle;
	A(6+HuboVPBody::eLAR, 6+HuboVPBody::eLAR) = 2*weightTrackAnkle;
	A(6+HuboVPBody::eLAP, 6+HuboVPBody::eLAP) = 2*weightTrackAnkle;

	A(6+HuboVPBody::eRSP, 6+HuboVPBody::eRSP) = 2*weightTrackUpper;
	A(6+HuboVPBody::eRSR, 6+HuboVPBody::eRSR) = 2*weightTrackUpper;
	A(6+HuboVPBody::eRSY, 6+HuboVPBody::eRSY) = 2*weightTrackUpper;
	A(6+HuboVPBody::eREB, 6+HuboVPBody::eREB) = 2*weightTrackUpper;
	A(6+HuboVPBody::eRWP, 6+HuboVPBody::eRWP) = 2*weightTrackUpper;
	A(6+HuboVPBody::eRWY, 6+HuboVPBody::eRWY) = 2*weightTrackUpper;
	A(6+HuboVPBody::eLSP, 6+HuboVPBody::eLSP) = 2*weightTrackUpper;
	A(6+HuboVPBody::eLSR, 6+HuboVPBody::eLSR) = 2*weightTrackUpper;
	A(6+HuboVPBody::eLSY, 6+HuboVPBody::eLSY) = 2*weightTrackUpper;
	A(6+HuboVPBody::eLEB, 6+HuboVPBody::eLEB) = 2*weightTrackUpper;
	A(6+HuboVPBody::eLWP, 6+HuboVPBody::eLWP) = 2*weightTrackUpper;
	A(6+HuboVPBody::eLWY, 6+HuboVPBody::eLWY) = 2*weightTrackUpper;
	if(bCalCP)
	{
		A.block(0, 0, 32, 32) += 2 * (R.transpose()*R);
		//A.block(0, 0, 32, 32) += 2 * (S.transpose() * S);
		A.block(0, 32, 32, Jsup.rows()) = Jsup.transpose();
		A.block(32, 0, Jsup.rows(), 32) = Jsup;
	}

	//b.resize(32);
	b.resize(32+Jsup.rows());
	b.head(32) = A.block(0,0,32,32)*desDofAccel;
	if(bCalCP)
	{
		b.head(32) += 2 * (R.transpose() * (LdotDes - rbias));
		//b.head(32) += 2 * (S.transpose() * (HdotDes - sbias));
		b.tail(Jsup.rows()) = -dJsup * dofVels;
	}
	//std::cout << "b: " << b.transpose() << std::endl;

	Eigen::VectorXd ddth = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
	Eigen::Vector3d rootAcc = ddth.head(3);
	Eigen::Vector3d rootAngAcc = ddth.segment(3, 3);
	Eigen::VectorXd otherJointDdth = ddth.segment(6, 26);
	//std::cout << "ddth: " << ddth.transpose() << std::endl;
	//std::cout << "real v:" << (M*J*ddth + rs).head(3).transpose() << std::endl;
	//std::cout << "des  v:" << LdotDes.head(3).transpose() << std::endl;

	//std::cout << (ddth-desDofAccel).norm() << std::endl;

	huboVpBody->applyRootJointDofAccel(rootAcc, rootAngAcc);
	huboVpBody->applyAllJointDofAccel(otherJointDdth);
}

void HuboVpController::comTrackingWithoutPhysics(
	double comx, double comy, double comz
	)
{
	const int frame = huboMotion->getCurrentFrame();

	comTrackPoint = Vector3d(comx, comy, comz);

	std::vector<Joint*> &joints = huboVpBody->pHuboMotion->joints;
	Eigen::VectorXd dangles;

	double angle;

	for(int j=0; j < 2000
		&& computePenaltyWithCom(constraints, constraintPosition, constraintOrientation) ; j++)
	{

		comSolve(dangles, 100, constraints, constraintPosition, constraintOrientation);

		
		{
			// For Hip Translation and Orientation
			Vector3d rootTrans;
			Quaterniond rootQuater;
			double xangle, yangle, zangle;

			rootTrans = dangles.segment(0,3) + joints[0]->getTranslation(frame);
			xangle = dangles[3];
			yangle = dangles[4];
			zangle = dangles[5];
			rootQuater = Quaterniond( cos(xangle/2),sin(xangle/2),0,0) 
				* Quaterniond( cos(yangle/2),0,sin(yangle/2),0) 
				* Quaterniond( cos(zangle/2),0,0,sin(zangle/2)) 
				* joints[0]->getRotation(frame);

			joints[0]->setTranslation(frame, rootTrans);
			joints[0]->setRotation(frame, rootQuater);
		}

		for(int i=1; i < joints.size(); i++)
		{
			// other bodies
			angle = joints[i]->getAngle(frame) + dangles[i+5]; // +5 for root DOF is 6

			if(angle < joints[i]->constraintAngle[0])
				angle = joints[i]->constraintAngle[0];
			else if(angle > joints[i]->constraintAngle[1])
				angle = joints[i]->constraintAngle[1];
			Quaterniond q = 
				Quaterniond(Eigen::AngleAxisd(angle, joints.at(i)->constraintAxis));

			joints.at(i)->setRotation(frame, q);

		}
	}
}


void HuboVpController::comSolve(
	Eigen::VectorXd &dAngles, 
	int nStep, 
	std::vector<Joint*> &constraints, 
	std::vector<Vector3d, aligned_allocator<Vector3d> > &constraintPosition,
	std::vector<Quaterniond, aligned_allocator<Quaterniond> > &constraintOrientation)
{
	const int frame = huboMotion->getCurrentFrame();
	// solve Jacobian
	Eigen::MatrixXd jacobian, Jt, Jp;
	Eigen::MatrixXd massMatrix;
	Eigen::VectorXd dcom, dcomp;
	
	huboVpBody->pIKSolver->computeFullJacobian();

	Vector3d com = huboVpBody->pHuboMotion->getHuboComGlobalPosition();
	
	dcom.resize(3+6*constraints.size());
	
	for(int i=0; i < 3; i++)
		dcom(i) = (comTrackPoint[i] - com[i])*huboVpBody->pHuboMotion->getHuboMass()/(nStep);
	

	Vector3d v;
	for(int i=0; i < constraints.size(); i++)
	{
		//Translation Parts
		v = constraintPosition[i] - constraints[i]->getGlobalComPosition(frame);
		for(int j=0; j < 3; j++)
			dcom(3+6*i+j) = v[j]/nStep;

		//Orientation Parts
		Quaterniond ori=constraints[i]->getGlobalOrientation(frame);
		v = diffQuat(constraintOrientation[i], ori);
		for(int j=0; j<3; j++)
			dcom(3+6*i+3+j) = v[j]/nStep;
	}
	
	huboVpBody->pHuboMotion->getHuboAllMassMatrix(massMatrix, 0);
	jacobian = massMatrix * huboVpBody->pIKSolver->fullJacobian;

	Eigen::MatrixXd constraintJacobian;
	for(int i=0; i<constraints.size(); i++)
	{
		huboVpBody->pIKSolver->computeConstraintFullJacobian(constraints[i],constraintJacobian);
		matrixStackUptoDown(jacobian, constraintJacobian);
	}
	dAngles = jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(dcom);
}

int HuboVpController::computePenaltyWithCom(
	std::vector<Joint*> &constraints, 
	std::vector<Vector3d, aligned_allocator<Vector3d> > &constraintPosition,
	std::vector<Quaterniond, aligned_allocator<Quaterniond> > &constraintOrientation
	)
{
	const int frame = huboMotion->getCurrentFrame();
	int result = (huboVpBody->pHuboMotion->getHuboComGlobalPosition(frame) - comTrackPoint).norm() >COMTHRESH;

	for(int i=0; i < constraints.size() ; i++)
		result += (constraints[i]->getGlobalComPosition(frame) - constraintPosition[i]).norm() > CONSTRAINTTHRESH;
	
	//TODO : 
	// orientation constraint CONSTRAINTANGULARTHRESH

	for(int i=0; i < constraints.size() ; i++)
		result += constraints[i]->getGlobalOrientation(frame).angularDistance(constraintOrientation[i]) > CONSTRAINTANGULARTHRESH;

	return result;
}


void HuboVpController::getConstraintStates()
{
	int frame = huboMotion->getCurrentFrame();
	constraints.clear();
	constraintPosition.clear();
	constraintOrientation.clear();

	if(leftHandHoldCheck)
	{
		constraints.push_back(huboVpBody->pHuboMotion->jointMap["LWP"]);
		constraintPosition.push_back(huboVpBody->pHuboMotion->jointMap["LWP"]->getGlobalComPosition(frame));
		constraintOrientation.push_back(huboVpBody->pHuboMotion->jointMap["LWP"]->getGlobalOrientation(frame));
	}
	if(leftFootHoldCheck)
	{
		constraints.push_back(huboVpBody->pHuboMotion->jointMap["LAR"]);
		constraintPosition.push_back(huboVpBody->pHuboMotion->jointMap["LAR"]->getGlobalComPosition(frame));
		constraintOrientation.push_back(huboVpBody->pHuboMotion->jointMap["LAR"]->getGlobalOrientation(frame));
	}
	if(rightHandHoldCheck)
	{
		constraints.push_back(huboVpBody->pHuboMotion->jointMap["RWP"]);
		constraintPosition.push_back(huboVpBody->pHuboMotion->jointMap["RWP"]->getGlobalComPosition(frame));
		constraintOrientation.push_back(huboVpBody->pHuboMotion->jointMap["RWP"]->getGlobalOrientation(frame));
	}
	if(rightFootHoldCheck)
	{
		constraints.push_back(huboVpBody->pHuboMotion->jointMap["RAR"]);
		constraintPosition.push_back(huboVpBody->pHuboMotion->jointMap["RAR"]->getGlobalComPosition(frame));
		constraintOrientation.push_back(huboVpBody->pHuboMotion->jointMap["RAR"]->getGlobalOrientation(frame));
	}
}
