#include "hubotrackingviewer.h"
#include "hubotrackingmanage.h"
#include "hubocmamanage.h"
#include <UniformBspline.h>
#include <HpMotionMath.h>

static HuboVpController *huboCont;
static HuboMotionData *referMotion;

HuboTrackingViewer::HuboTrackingViewer(QWidget *parent)
{
}

static double fitfunc(const double *x, int dim)
{
    double rhpOffset = -M_PI / 12;
    double rapOffset = -M_PI / 20;
    double fitness = 0;
    double Ks = huboCont->ks;
    double Kd = huboCont->kd;

    //huboCont->huboVpBody->pHuboMotion->setMotionSize(referMotion->getMotionSize());
    //huboCont->huboVpBody->pHuboMotion->setFrameRate(referMotion->getFrameRate());

    huboCont->initController();

    // set hybrid dynamics with floating base
    huboCont->huboVpBody->initHybridDynamics(true);

    // loop for entire time
    int totalStep = (int)
        (
        referMotion->getFrameTime()
        * referMotion->getMotionSize()
        /huboCont->timestep
        );
    double time;
    double framestep = 0;
    double frameTime = referMotion->getFrameTime();

    Eigen::VectorXd angleRefer, angleVelRefer, angleAccelRefer, desAccel;
    Eigen::VectorXd angle, angleVel, torques;
    Eigen::VectorXd angleOffset;
    Eigen::VectorXd angleOffsetWhenNonPeriodic;

    Eigen::Vector3d hipPosRefer, hipVelRefer, hipAccelRefer, hipDesAccel;
    Eigen::Vector3d hipPos, hipVel;

    Eigen::Vector3d hipAngVelRefer, hipAngAccelRefer, hipDesAngAccel;
    Eigen::Vector3d hipAngVel;
    Eigen::Quaterniond hipOrienRefer;
    Eigen::Quaterniond hipOrien;

    Vec3 foot1init = huboCont->huboVpBody->Foot[0]->GetFrame().GetPosition();
    Vec3 foot2init = huboCont->huboVpBody->Foot[1]->GetFrame().GetPosition();

    BezierSpline angleOffsetSpline;

    angleOffsetSpline.clear();

    angleOffset.resize(26);
    for (int i = 0; i < 3; i++)
    {
        angleOffset.setZero();

        angleOffset(HuboVPBody::eLHY) = x[12*i];
        angleOffset(HuboVPBody::eLHR) = x[12*i+1];
        angleOffset(HuboVPBody::eLHP) = x[12*i+2];
        angleOffset(HuboVPBody::eLKN) = x[12*i+3];
        angleOffset(HuboVPBody::eLAP) = x[12*i+4];
        angleOffset(HuboVPBody::eLAR) = x[12*i+5];

        angleOffset(HuboVPBody::eRHY) = x[12*i+6];
        angleOffset(HuboVPBody::eRHR) = x[12*i+7];
        angleOffset(HuboVPBody::eRHP) = x[12*i+8];
        angleOffset(HuboVPBody::eRKN) = x[12*i+9];
        angleOffset(HuboVPBody::eRAP) = x[12*i+10];
        angleOffset(HuboVPBody::eRAR) = x[12*i+11];

        angleOffsetSpline.setControlPoint(i, angleOffset);
        if (i == 0)
            angleOffsetSpline.setControlPoint(i+3, angleOffset);
    }

    angleOffsetWhenNonPeriodic.resize(26);
    angleOffsetWhenNonPeriodic.setZero();
    {
        //std::cout << huboCont->huboVpBody->vptohuboJointmap[huboCont->huboVpBody->joints.at(HuboVPBody::eLHP)]->name;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLHY) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLHR) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLHP) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLKN) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLAP) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLAR) = 0;

        angleOffsetWhenNonPeriodic(HuboVPBody::eRHY) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRHR) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRHP) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRKN) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRAP) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRAR) = 0;

    }

    double phase = 0;
	double timeBetFrame = 0;
	int frame = 0;
    huboCont->huboVpBody->setInitialHuboHipFromMotion(referMotion);
	huboCont->huboVpBody->setInitialHuboAngleFromMotion(referMotion);
	huboCont->huboVpBody->setInitialHuboAngleRateFromMotion(referMotion);
    Eigen::Vector3d rInitial = referMotion->getHuboComGlobalPositionInTime(0);
    Eigen::Vector3d vInitial = huboCont->huboVpBody->getCOMposition();

	double jointFit = 0, torqueFit = 0, velFit = 0, dirFit = 0, heightFit = 0, footToComFit = 0;

    for (int i = 0; i <= totalStep; i++)
    {
        time = i * huboCont->timestep;
        framestep += huboCont->timestep;

		frame = referMotion->timeToFrame(time);
		timeBetFrame = referMotion->timeToTimeBetweenFrame(time);

        // get desired acceleration for instance time
		vpRJoint v;
        hipPosRefer = referMotion->getHipJointGlobalPositionInTime(time);
        hipOrienRefer = referMotion->getHipJointGlobalOrientationInTime(time);
        hipVelRefer = referMotion->getHipJointVelInHuboMotionInTime(time);
        hipAngVelRefer = referMotion->getHipJointAngVelInHuboMotionInTime(time);
        hipAccelRefer = referMotion->getHipJointAccelInHuboMotionInTime(time);
        hipAngAccelRefer = referMotion->getHipJointAngAccelInHuboMotionInTime(time);

        hipPos = Vec3Tovector(huboCont->huboVpBody->Hip->GetFrame().GetPosition());
        hipVel = Vec3Tovector(huboCont->huboVpBody->Hip->GetLinVelocity(Vec3(0,0,0)));
        hipOrien = huboCont->huboVpBody->getOrientation(huboCont->huboVpBody->Hip);
        hipAngVel = Vec3Tovector(huboCont->huboVpBody->Hip->GetAngVelocity());

        referMotion->getAllAngleInHuboMotionInTime(time, angleRefer);
        phase = referMotion->getPeriodPhaseInTime(time);
		
        if (phase >= 0)
            angleRefer += angleOffsetSpline.getValue(phase);
        else
            angleRefer += angleOffsetWhenNonPeriodic;
        /*
        if ((phase = referMotion->getPeriodPhaseInTime(time)) >=0)
            angleRefer += angleOffsetSpline.getValue(phase);
        else
        {
            //TODO:
            // balancing

        }
        */
        referMotion->getAllAngleRateInHuboMotionInTime(time, angleVelRefer);
        referMotion->getAllAngleAccelInHuboMotionInTime(time, angleAccelRefer);

        huboCont->huboVpBody->getAllAngle(angle);
        huboCont->huboVpBody->getAllAngularVelocity(angleVel);

        // TODO:
        huboCont->getDesiredDofAccelForRoot(hipPosRefer, hipPos, hipVelRefer, hipVel, hipAccelRefer, hipDesAccel);
        huboCont->getDesiredDofAngAccelForRoot(hipOrienRefer, hipOrien, hipAngVelRefer, hipAngVel, hipAngAccelRefer, hipDesAngAccel);

        huboCont->getDesiredDofAccel(angleRefer, angle, angleVelRefer, angleVel, angleAccelRefer, desAccel);

        // set acceleration to joint
        huboCont->huboVpBody->applyRootJointDofAccel(hipDesAccel, hipDesAngAccel);
        huboCont->huboVpBody->applyAllJointDofAccel(desAccel);
        //std::cout << desAccel.transpose() << std::endl;

        huboCont->huboVpBody->solveHybridDynamics();

        huboCont->huboVpBody->getAllJointTorque(torques);

        // go one time step
        huboCont->stepAheadWithPenaltyForces();

        //TODO:
        //set appropriate objective term

        //end-effector term
        //Vec3 hand1 = huboCont->huboVpBody->Hand[0]->GetFrame().GetPosition();
        //Vec3 hand2 = huboCont->huboVpBody->Hand[0]->GetFrame().GetPosition();

        //foot term
        //Vec3 foot1 = hubo->huboVpBody->Foot[0]->GetFrame().GetPosition();
        //Vec3 foot2 = hubo->huboVpBody->Foot[1]->GetFrame().GetPosition();
        //fitness += (foot1[0]-foot1init[0])*(foot1[0]-foot1init[0]);
        //fitness += (foot2[0]-foot2init[0])*(foot2[0]-foot2init[0]);

        // stance foot term
        //fitness += (foot2[1]-foot2init[1])*(foot2[1]-foot2init[1]);

		//joint term
		Eigen::VectorXd rAngles;
		referMotion->getAllAngleInHuboMotionInTime(time, rAngles);
		huboCont->huboVpBody->getAllAngle(angle);

		jointFit += (rAngles - angle).squaredNorm();

        //torque term
        torqueFit += 0.00001*torques.squaredNorm();

        //velocity term
		Eigen::Vector3d vrcom = referMotion->getHipJointVelInHuboMotionInTime(time);
		Eigen::Vector3d vpcom = huboCont->huboVpBody->getCOMvelocity();
		vrcom.y() = 0;
		vpcom.y() = 0;
		velFit += abs(vrcom.squaredNorm() - vpcom.squaredNorm());

		//direction term
		Eigen::Vector3d vDir  = huboCont->huboVpBody->getCOMvelocity().normalized();
		Eigen::Vector3d vrDir = (huboCont->huboVpBody->getCOMposition() - vInitial).normalized();

        dirFit += 1-vDir.dot(vrDir) ;

        //direction term
        //fitness += 20*(hubo->huboVpBody->getHipDirection()-Vector3d(0,0,1)).squaredNorm();


		//foot to com term
		
		Eigen::Vector3d footToCom(0,0,0), refFootToCom(0,0,0);
		if (phase >= 0 && phase <= 0.5)
		{
			footToCom = Vec3Tovector(
				huboCont->huboVpBody->Foot[1]->GetFrame().GetPosition()
				- huboCont->huboVpBody->getCOM() 
				);
			refFootToCom =
				referMotion->jointMap["LAR"]->getGlobalBoundingBoxPosition(frame, timeBetFrame)
				- referMotion->getHuboComGlobalPositionInTime(time);
		}
		else if (phase > 0.5)
		{
			footToCom = Vec3Tovector(
				huboCont->huboVpBody->Foot[0]->GetFrame().GetPosition()
				- huboCont->huboVpBody->getCOM() 
				);
			refFootToCom =
				referMotion->jointMap["RAR"]->getGlobalBoundingBoxPosition(frame, timeBetFrame)
				- referMotion->getHuboComGlobalPositionInTime(time);
		}

		footToComFit += (footToCom - refFootToCom).squaredNorm();
		
        //height term
        double dy =
            (
            huboCont->huboVpBody->getCOMposition().y()
            - referMotion->getHuboComGlobalPositionInTime(time).y()
            );

        if (abs(dy) > 0.3)
        {
			jointFit += (totalStep - i) *(rAngles - angle).squaredNorm();
			torqueFit += (totalStep - i) *0.00001*torques.squaredNorm();
			velFit += (totalStep - i) *abs(vrcom.squaredNorm() - vpcom.squaredNorm());
			dirFit += (totalStep - i) *1-vDir.dot(vrDir) ;
			footToComFit += (totalStep - i) *(footToCom - refFootToCom).squaredNorm();

            heightFit += (totalStep-i) * dy*dy;

            //heightFit += (totalStep-i)*(totalStep-i);
            break;
            //fitness += (totalStep-i) * abs(dy);
            //fitness += dy*dy;
        }
    }
    // goal position term
    //fitness += RES * (hubo->huboVpBody->getCOMposition()-Vector3d(0, 0.7, 0.25)).squaredNorm();

    // goal direction term
    //fitness += RES*(hubo->huboVpBody->getHipDirection()-Vector3d(0,0,1)).squaredNorm();

    // goal Hip orientation term
    //double angleDist =
    //	hubo->huboVpBody->getOrientation(hubo->huboVpBody->Hip)
    //	.angularDistance(Quaterniond(1, 0, 0, 0));
    //fitness += RES * angleDist*angleDist;

    //// foot goal term
    //Vec3 foot1final = hubo->huboVpBody->Foot[0]->GetFrame().GetPosition();
    //Vec3 foot2final = hubo->huboVpBody->Foot[1]->GetFrame().GetPosition();
    //fitness += RES*(foot1final[2] - 0.6) *(foot1final[2]-0.6);
    //fitness += RES*(foot2final[2] - 0.0) *(foot2final[2]-0.0);

    //// swing foot term
    //fitness += RES*(foot1final[1] - foot1init[1]) *(foot1final[1]-foot1init[1]);
	fitness = 0
		+ 1.0	*jointFit
		+ 1.0	*torqueFit
		+ 1.0	*velFit
		+ 0.05	*dirFit
		+ 1.0 * heightFit
		+ 1 * footToComFit
		;

	/*
	std::cout << "joint : " << 0.5*jointFit <<
		" torque : " << torqueFit <<
		" vel : " << 0.5*velFit <<
		" dir : " << 0.05*dirFit <<
		" height : " << 20*heightFit << 
		" footToCom : " << 1*footToComFit << 
		std::endl;
		//*/

    return fitness;
}


void HuboTrackingViewer::useLatestCmaResult(char* filename, std::vector<double> &solution)
{
    std::ifstream fin;
    int dim;
    double sol;
    fin.open(filename);
    fin >> dim;
    for(int i=0; i<dim; i++)
    {
        fin >> sol;
        solution.push_back(sol);
    }
    fin.close();
}

void HuboTrackingViewer::cmaRun(int maxIter, int useLatestResult)
{
    if (referMotion == NULL)
    {
        std::cout << "Reference motion is empty. Please set a reference motion.(using setReferMotion())" << std::endl;
        return;
    }
    huboCont = this->hubo;
    std::vector<double> xstart, xdev, ub, lb;

    int dim = 36;

    if (useLatestResult == 1)
    {
        useLatestCmaResult("../CmaData/trackingCmaSolution.txt", xstart);
        for (int i = 0; i < dim; i++)
            xdev.push_back(M_PI/8);
    }
    else
    {
        for (int i = 0; i < dim; i++)
        {
            xstart.push_back(0.0);
            xdev.push_back(M_PI/4);
        }
    }
	for (int i = 0; i < dim; i++)
	{
		ub.push_back(M_PI/2);
		lb.push_back(-M_PI/2);
	}

	/*
    for (int i = 0; i < 3; i++)
    {
        ub.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LHY]->constraintAngle[1]);
        ub.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LHR]->constraintAngle[1]);
        ub.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LHP]->constraintAngle[1]);
        ub.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LKN]->constraintAngle[1]);
        ub.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LAP]->constraintAngle[1]);
        ub.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LAR]->constraintAngle[1]);

        lb.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LHY]->constraintAngle[0]);
        lb.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LHR]->constraintAngle[0]);
        lb.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LHP]->constraintAngle[0]);
        lb.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LKN]->constraintAngle[0]);
        lb.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LAP]->constraintAngle[0]);
        lb.push_back(hubo->huboVpBody->vptohuboJointmap[hubo->huboVpBody->LAR]->constraintAngle[0]);
    }
	*/

    //hubo->huboVpBody->getHuboLimit(0, lb);
    //hubo->huboVpBody->getHuboLimit(1, ub);

	cma.init();
    cma.setOptimizer(
        fitfunc,
        dim,
        0,
        xstart,
        xdev,
        lb,
        ub);
    cma.maxIteration = maxIter;

	cmaTh.setCmaOptimizer(&cma);
	cmaTh.start();

    //cma.run_boundary();
    //cma.run();

    //cma.saveSolution("../CmaData/trackingCmaSolution.txt");

    //for debug
    //for(auto it = cma.solution.begin(); it!=cma.solution.end(); it++)
}

void HuboTrackingViewer::setCmaMotion(int frameRate, int useManualSolution)
{
    double rhpOffset = -M_PI / 12;
    double rapOffset = -M_PI / 20;
    if (referMotion == NULL)
    {
        std::cout << "Reference motion is empty. Please set a reference motion.(using setReferMotion())" << std::endl;
        return;
    }
	double *x = new double [cma.solution.size()];
	for (int i = 0; i < cma.solution.size(); i++)
		x[i] = cma.solution.at(i);
	/*
	if (useManualSolution)
		x = manualSol;
	*/
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
        frameTime = 1.0/frameRate;

    Eigen::VectorXd angleRefer, angleVelRefer, angleAccelRefer, desAccel;
    Eigen::VectorXd angle, angleVel, torques;
    Eigen::VectorXd angleOffset;
    Eigen::VectorXd angleOffsetWhenNonPeriodic;

    Eigen::Vector3d hipPosRefer, hipVelRefer, hipAccelRefer, hipDesAccel;
    Eigen::Vector3d hipPos, hipVel;

    Eigen::Vector3d hipAngVelRefer, hipAngAccelRefer, hipDesAngAccel;
    Eigen::Vector3d hipAngVel;
    Eigen::Quaterniond hipOrienRefer;
    Eigen::Quaterniond hipOrien;

    Vec3 foot1init = hubo->huboVpBody->Foot[0]->GetFrame().GetPosition();
    Vec3 foot2init = hubo->huboVpBody->Foot[1]->GetFrame().GetPosition();

    BezierSpline angleOffsetSpline;

    angleOffsetSpline.clear();

    angleOffset.resize(26);
    for (int i = 0; i < 3; i++)
    {
        angleOffset.setZero();

        angleOffset(HuboVPBody::eLHY) = x[12*i];
        angleOffset(HuboVPBody::eLHR) = x[12*i+1];
        angleOffset(HuboVPBody::eLHP) = x[12*i+2];
        angleOffset(HuboVPBody::eLKN) = x[12*i+3];
        angleOffset(HuboVPBody::eLAP) = x[12*i+4];
        angleOffset(HuboVPBody::eLAR) = x[12*i+5];

        angleOffset(HuboVPBody::eRHY) = x[12*i+6];
        angleOffset(HuboVPBody::eRHR) = x[12*i+7];
        angleOffset(HuboVPBody::eRHP) = x[12*i+8];
        angleOffset(HuboVPBody::eRKN) = x[12*i+9];
        angleOffset(HuboVPBody::eRAP) = x[12*i+10];
        angleOffset(HuboVPBody::eRAR) = x[12*i+11];

        angleOffsetSpline.setControlPoint(i, angleOffset);
        if (i == 0)
            angleOffsetSpline.setControlPoint(i+3, angleOffset);
    }

    angleOffsetWhenNonPeriodic.resize(26);
    angleOffsetWhenNonPeriodic.setZero();
    {
        angleOffsetWhenNonPeriodic(HuboVPBody::eLHY) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLHR) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLHP) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLKN) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLAP) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLAR) = 0;

        angleOffsetWhenNonPeriodic(HuboVPBody::eRHY) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRHR) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRHP) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRKN) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRAP) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRAR) = 0;

    }
    double phase = 0;
    hubo->huboVpBody->setInitialHuboHipFromMotion(referMotion);
	huboCont->huboVpBody->setInitialHuboAngleFromMotion(referMotion);
	huboCont->huboVpBody->setInitialHuboAngleRateFromMotion(referMotion);

    for (int i = 0; i <= totalStep; i++)
    {
        time = i * hubo->timestep;
        framestep += hubo->timestep;

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
        phase = referMotion->getPeriodPhaseInTime(time);
        if (phase >= 0)
            angleRefer += angleOffsetSpline.getValue(phase);
        else
            angleRefer += angleOffsetWhenNonPeriodic;

        /*
        if ((phase = referMotion->getPeriodPhaseInTime(time)) >=0)
            angleRefer += angleOffsetSpline.getValue(phase);
        else
        {
            //TODO:
            // balancing

        }
        */
        referMotion->getAllAngleRateInHuboMotionInTime(time, angleVelRefer);
        referMotion->getAllAngleAccelInHuboMotionInTime(time, angleAccelRefer);

        hubo->huboVpBody->getAllAngle(angle);
        hubo->huboVpBody->getAllAngularVelocity(angleVel);

        // TODO:
        hubo->getDesiredDofAccelForRoot(hipPosRefer, hipPos, hipVelRefer, hipVel, hipAccelRefer, hipDesAccel);
        hubo->getDesiredDofAngAccelForRoot(hipOrienRefer, hipOrien, hipAngVelRefer, hipAngVel, hipAngAccelRefer, hipDesAngAccel);

        hubo->getDesiredDofAccel(angleRefer, angle, angleVelRefer, angleVel, angleAccelRefer, desAccel);

        // set acceleration to joint
        hubo->huboVpBody->applyRootJointDofAccel(hipDesAccel, hipDesAngAccel);
        hubo->huboVpBody->applyAllJointDofAccel(desAccel);
        //std::cout << desAccel.transpose() << std::endl;

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
			//height term
			//double dy =
			//	(
			//	huboCont->huboVpBody->getCOMposition().y()
			//	- referMotion->getHuboComGlobalPositionInTime(time).y()
			//	);
			// std::cout << hubo->huboVpBody->pHuboMotion->getCurrentFrame() << " " << dy << std::endl;
        }


    }
	adjustHuboMotionToViewer();
}

void HuboTrackingViewer::setReferMotion(HuboMotionData *refer)
{
    referMotion = refer;
	/*
    HuboTrackingManage *huboTrMan = new HuboTrackingManage;
	huboTrMan->init(this);
    huboTrMan->move(200+width()+20, 200);
	huboTrMan->setWindowTitle(QString("Manual Solution"));
    huboTrMan->show();
	*/
    HuboCmaManage *huboTrMan = new HuboCmaManage;
	huboTrMan->initManager(this);
    huboTrMan->move(200+width()+20, 200);
	huboTrMan->setWindowTitle(QString("Cma Dialog"));
    huboTrMan->show();
}
