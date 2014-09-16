#include "huboikviewer.h"
#include <UniformBspline.h>
#include <HpMotionMath.h>
#include <IKSolver.h>
#include "huboikmanage.h"

static HuboVpController *huboCont;
static HuboMotionData *referMotion;

HuboIkViewer::HuboIkViewer(QWidget *parent)
{
	hubo = new HuboVpController;
	hubo->initController();

	HuboIkManage *man = new HuboIkManage;
	man->init(this);
	man->show();
}

void HuboIkViewer::rFoot()
{
	HuboMotionData *data = hubo->huboVpBody->pHuboMotion;
	IKSolver *ik = new IKSolver(hubo->huboVpBody->pHuboMotion);

	Eigen::Vector3d p = data->jointMap["RAR"]->getGlobalPosition(data->getCurrentFrame());
	Quaterniond q(1, 0, 0, 0);
	ik->run_r("RAR", p, q);

	delete ik;
}