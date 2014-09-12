#include "hubotrackingviewer.h"
#include "hubotrackingmanage.h"
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
    huboCont->huboVpBody->pHuboMotion->setFrameRate(referMotion->getFrameRate());

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

        angleOffset(HuboVPBody::eLHY) = x[6*i];
        angleOffset(HuboVPBody::eLHR) = x[6*i+1];
        angleOffset(HuboVPBody::eLHP) = x[6*i+2];
        angleOffset(HuboVPBody::eLKN) = x[6*i+3];
        angleOffset(HuboVPBody::eLAP) = x[6*i+4];
        angleOffset(HuboVPBody::eLAR) = x[6*i+5];

        angleOffset(HuboVPBody::eRHY) = -x[6*i];
        angleOffset(HuboVPBody::eRHR) = -x[6*i+1];
        angleOffset(HuboVPBody::eRHP) = x[6*i+2];
        angleOffset(HuboVPBody::eRKN) = x[6*i+3];
        angleOffset(HuboVPBody::eRAP) = x[6*i+4];
        angleOffset(HuboVPBody::eRAR) = -x[6*i+5];

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
        angleOffsetWhenNonPeriodic(HuboVPBody::eLHP) = 0+rhpOffset;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLKN) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLAP) = 0+rapOffset;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLAR) = 0;

        angleOffsetWhenNonPeriodic(HuboVPBody::eRHY) = -0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRHR) = -0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRHP) =  0+rhpOffset;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRKN) =  0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRAP) =  0+rapOffset;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRAR) = -0;

    }

    double phase = 0;
    double referInitialX = referMotion->getHuboComGlobalPositionInTime(0).x();
    huboCont->huboVpBody->setInitialHuboHipFromMotion(referMotion);

    for (int i = 0; i <= totalStep; i++)
    {
        time = i * huboCont->timestep;
        framestep += huboCont->timestep;

        // get desired acceleration for instance time
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


        //velocity term
        //fitness += (hubo->huboVpBody->getCOMvelocity() - Vector3d(0, 0, 0.4)).squaredNorm();

        //height term
        double dy =
            (
            huboCont->huboVpBody->getCOMposition().y()
            - referMotion->getHuboComGlobalPositionInTime(time).y()
            );

        if (abs(dy) > 0.3)
        {
            fitness += (totalStep-i) * dy*dy;
            break;
            //fitness += (totalStep-i) * abs(dy);
            //fitness += dy*dy;
        }

        //fitness += 1*dy*dy;
        //fitness += 1 * abs(dy);

        double dx =
            (
            huboCont->huboVpBody->getCOMposition().x()
            - (referMotion->getHuboComGlobalPositionInTime(time).x()-referInitialX)
            );

        fitness += 0.1*dx*dx;


        //direction term
        //fitness += 20*(hubo->huboVpBody->getHipDirection()-Vector3d(0,0,1)).squaredNorm();

        //torque term
        fitness += 0.00001*torques.squaredNorm();
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

void HuboTrackingViewer::cmaRun(int maxIter)
{
    if (referMotion == NULL)
    {
        std::cout << "Reference motion is empty. Please set a reference motion.(using setReferMotion())" << std::endl;
        return;
    }
    huboCont = this->hubo;
    std::vector<double> xstart, xdev, ub, lb;

    int dim = 18;
    int useLatestResult = 1;

    if (useLatestResult == 1)
    {
        useLatestCmaResult("../CmaData/trackingCmaSolution.txt", xstart);
        for (int i = 0; i < dim; i++)
            xdev.push_back(0.01);
    }
    else
    {
        for (int i = 0; i < dim; i++)
        {
            xstart.push_back(0.0);
            xdev.push_back(0.1);
        }
    }

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

    //hubo->huboVpBody->getHuboLimit(0, lb);
    //hubo->huboVpBody->getHuboLimit(1, ub);

    cma.setOptimizer(
        fitfunc,
        dim,
        0,
        xstart,
        xdev,
        lb,
        ub);
    cma.maxIteration = maxIter;

    cma.run_boundary();
    //cma.run();

    cma.saveSolution("../CmaData/trackingCmaSolution.txt");

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
    std::vector<double> &x = cma.solution;
	if (useManualSolution)
		x = manualSol;
	
    double Ks = hubo->ks;
    double Kd = hubo->kd;

    hubo->initController();

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

        angleOffset(HuboVPBody::eLHY) = x[6*i];
        angleOffset(HuboVPBody::eLHR) = x[6*i+1];
        angleOffset(HuboVPBody::eLHP) = x[6*i+2];
        angleOffset(HuboVPBody::eLKN) = x[6*i+3];
        angleOffset(HuboVPBody::eLAP) = x[6*i+4];
        angleOffset(HuboVPBody::eLAR) = x[6*i+5];

        angleOffset(HuboVPBody::eRHY) = -x[6*i];
        angleOffset(HuboVPBody::eRHR) = -x[6*i+1];
        angleOffset(HuboVPBody::eRHP) = x[6*i+2];
        angleOffset(HuboVPBody::eRKN) = x[6*i+3];
        angleOffset(HuboVPBody::eRAP) = x[6*i+4];
        angleOffset(HuboVPBody::eRAR) = -x[6*i+5];

        angleOffsetSpline.setControlPoint(i, angleOffset);
        if (i == 0)
            angleOffsetSpline.setControlPoint(i+3, angleOffset);
    }

    angleOffsetWhenNonPeriodic.resize(26);
    angleOffsetWhenNonPeriodic.setZero();
    {
        angleOffsetWhenNonPeriodic(HuboVPBody::eLHY) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLHR) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLHP) = 0+rhpOffset;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLKN) = 0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLAP) = 0+rapOffset;
        angleOffsetWhenNonPeriodic(HuboVPBody::eLAR) = 0;

        angleOffsetWhenNonPeriodic(HuboVPBody::eRHY) = -0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRHR) = -0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRHP) =  0+rhpOffset;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRKN) =  0;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRAP) =  0+rapOffset;
        angleOffsetWhenNonPeriodic(HuboVPBody::eRAR) = -0;

    }
    double phase = 0;
    hubo->huboVpBody->setInitialHuboHipFromMotion(referMotion);

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
        }
    }
	adjustHuboMotionToViewer();
}

void HuboTrackingViewer::setReferMotion(HuboMotionData *refer)
{
    referMotion = refer;
    HuboTrackingManage huboTrMan;
    huboTrMan.exec();
}
