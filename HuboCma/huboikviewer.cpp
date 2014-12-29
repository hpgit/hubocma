#include "huboikviewer.h"
//#include <UniformBspline.h>
#include <HpMotionMath.h>
#include <IKSolver.h>
#include "huboikmanage.h"

HuboIkViewer::HuboIkViewer(QWidget *parent)
{
	HuboIkManage *man = new HuboIkManage;
	man->init(this);
    man->move(200+width()+20, 200);
	man->show();
}
void HuboIkViewer::setReferMotion(HuboMotionData *_refer)
{
	refer = _refer;
}

void HuboIkViewer::rFoot()
{
	HuboMotionData *data = hubo->getHuboMotion();
	IKSolver *ik = new IKSolver(data);

	Eigen::Vector3d p = refer->jointMap["RAR"]->getGlobalBoundingBoxPosition(data->getCurrentFrame()) + Vector3d(0,0.1,0);

	ik->maxIter = 400;
	ik->ikEps = 0.00001;
	ik->weightAng = 10;
	ik->weightPos = 1;
	ik->stepSize = 0.01;
	
	Quaterniond q;
	q = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY());
	ik->run_r("RAR", p, q);
	//ik->run_gradient_descent("RAR", p, q);

	delete ik;
	glWidget->updateGL();
}

void HuboIkViewer::solve(std::string name, Eigen::Vector3d &dpos, bool parallel,
	int maxIter, double ikEps, double weightPos, double weightAng, double stepSize)
{
	HuboMotionData *data = hubo->getHuboMotion();
	IKSolver *ik = new IKSolver(data);

	Eigen::Vector3d p = refer->jointMap[name]->getGlobalBoundingBoxPosition(data->getCurrentFrame()) + dpos;
	Quaterniond q(1,0,0,0);
	ik->maxIter = maxIter;
	ik->ikEps = ikEps;
	ik->weightAng = weightAng;
	ik->weightPos = weightPos;
	ik->stepSize = stepSize;

	std::cout << "solve IK!" << std::endl;
	Quaterniond qqq= data->jointMap[name]->getGlobalOrientation(data->getCurrentFrame());
	std::cout << qqq.w() << " " << qqq.vec().transpose() << std::endl;

	if (parallel)
	{
		//TODO:
		Quaterniond qq = refer->jointMap[name]->getGlobalOrientation(data->getCurrentFrame());
        Quaterniond qv(0, 0,0,1);
        Vector3d vv = (qq*qv*qq.inverse()).vec();
		std::cout << vv.transpose() << std::endl;
        Vector3d yy(0,1,0);
        q = Quaterniond(Eigen::AngleAxisd(atan2(vv.x(), vv.z()), yy));
		std::cout << q.w() << " " << q.vec().transpose() << std::endl;
	}
	else
	{
		q = refer->jointMap[name]->getGlobalOrientation(data->getCurrentFrame());
	}

	ik->run_r(name, p, q);

	qqq= data->jointMap[name]->getGlobalOrientation(data->getCurrentFrame());
	std::cout << qqq.w() << " " << qqq.vec().transpose() << std::endl;

	delete ik;
	glWidget->updateGL();
}
