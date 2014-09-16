#include "hubomaincontroller.h"
#include "hubotrackingviewer.h"
#include "huboikviewer.h"
#include <QApplication>
#include <HuboVpController.h>
#include <CmaOptimizer.h>
#include <UniformBspline.h>
#ifdef WIN32
#include <ctime>
#endif

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

	if (false)
	{
		HuboVpController *hubo, *huboRefer;
		//for tracking
		CmaOptimizer cma;
		hubo = new HuboVpController;
		hubo->setTimeStep(0.001);
		huboRefer = new HuboVpController;

		hubo->initController();

		huboRefer->huboVpBody->pHuboMotion->import("../CmaData/walk_repeated.txt", 35, 120);
		huboRefer->huboVpBody->pHuboMotion->importContactPeriodAnnotation("../CmaData/walk_repeated_contactPeriod.txt", 35, 120);
		HuboGlViewer *win = new HuboGlViewer;
		win->setWindowTitle(QString("Hubo Reference Motion"));
		win->init(huboRefer);
		//win->show();

		HuboTrackingViewer *contWin = new HuboTrackingViewer;
		contWin->setWindowTitle(QString("Hubo Tracking Viewer"));
		contWin->init(hubo);
		//contWin->setReferMotion(huboRefer->huboVpBody->pHuboMotion);
		contWin->move(200, 200);
		//contWin.cmaRun(300);
		//contWin.cma.loadSolution("../CmaData/trackingCmaSolution.txt");
		//contWin.setCmaMotion();
		//contWin.setInitMotion();
		//contWin->show();
	}

	if (false)
	{
		HuboVpController *hubo, *huboRefer;
		//for tracking
		CmaOptimizer cma;
		hubo = new HuboVpController;
		hubo->setTimeStep(0.001);
		huboRefer = new HuboVpController;

		hubo->initController();

		huboRefer->huboVpBody->pHuboMotion->import("../CmaData/n_wd2_WalkForwardFast05.txt", 10, 110);
		huboRefer->huboVpBody->pHuboMotion->importContactPeriodAnnotation("../CmaData/n_wd2_WalkForwardFast05_contactPeriod.txt", 10, 110);
		HuboGlViewer *win = new HuboGlViewer;
		win->setWindowTitle(QString("Hubo Reference Motion"));
		win->init(huboRefer);
		win->show();

		HuboTrackingViewer *contWin = new HuboTrackingViewer;
		contWin->setWindowTitle(QString("Hubo Tracking Viewer"));
		contWin->init(hubo);
		contWin->setReferMotion(huboRefer->huboVpBody->pHuboMotion);
		contWin->move(200, 200);
		//contWin.cmaRun(300);
		//contWin.cma.loadSolution("../CmaData/trackingCmaSolution.txt");
		//contWin.setCmaMotion();
		//contWin.setInitMotion();
		contWin->show();
	}

	if (true)
	{
		//for IK
		HuboVpController *hubo, *huboRefer;
		hubo = new HuboVpController;
		hubo->setTimeStep(0.001);
		huboRefer = new HuboVpController;

		hubo->initController();

		huboRefer->huboVpBody->pHuboMotion->import("../CmaData/n_wd2_WalkForwardFast05.txt", 10, 110);
		HuboGlViewer *win = new HuboGlViewer;
		win->setWindowTitle(QString("Hubo Reference Motion"));
		win->init(huboRefer);
		win->show();

		HuboIkViewer *contWin = new HuboIkViewer;
		hubo->huboVpBody->pHuboMotion->import("../CmaData/n_wd2_WalkForwardFast05.txt", 10, 110);
		contWin->setWindowTitle(QString("Hubo Inverse Kinematics Viewer"));
		contWin->init(hubo);
		contWin->move(200, 200);
		contWin->show();

	}

	if (0)
	{
		//for balancing
		HuboVpController *hubo, *huboRefer;
		hubo = new HuboVpController;
		hubo->setTimeStep(0.001);
		huboRefer = new HuboVpController;

		hubo->initController();

		huboRefer->huboVpBody->pHuboMotion->import("../CmaData/n_kick.txt", 35, 100);
		HuboGlViewer *win = new HuboGlViewer;
		win->setWindowTitle(QString("HuboReference Motion"));
		win->init(huboRefer);
		//win->show();

	}

    return a.exec();
}