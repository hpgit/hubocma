#include "hubomaincontroller.h"
#include "hubotrackingviewer.h"
#include <QApplication>
#include <HuboVpController.h>
#include <CmaOptimizer.h>
#include <UniformBspline.h>
#ifdef WIN32
#include <ctime>
#endif

HuboVpController *hubo, *huboRefer;
//HuboGlWindow *huboGl, *huboReferGl;
CmaOptimizer cma;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    hubo = new HuboVpController;
    hubo->setTimeStep(0.001);
    huboRefer = new HuboVpController;

    hubo->initController();

////huboRefer->huboVpBody->pHuboMotion->import("../CmaData/walk.txt");
    huboRefer->huboVpBody->pHuboMotion->import("../CmaData/walk_repeated.txt", 30, 120);
    huboRefer->huboVpBody->pHuboMotion->importContactPeriodAnnotation("../CmaData/walk_repeated_contactPeriod.txt", 30, 120);
    HuboGlViewer win;
    win.setWindowTitle(QString("Hubo Reference Motion"));
    win.init(huboRefer);
    win.show();

    HuboTrackingViewer contWin;
    contWin.setWindowTitle(QString("Hubo Reference Motion"));
    contWin.init(hubo);
    contWin.setReferMotion(huboRefer->huboVpBody->pHuboMotion);
    time_t start = time(NULL);
    contWin.cmaRun(300);
    //contWin.cma.loadSolution("../CmaData/trackingCmaSolution.txt");
    contWin.setCmaMotion();
    //contWin.setInitMotion();
    contWin.show();
    printf("cacluation time : %u", time(NULL) - start);

    return a.exec();
}
