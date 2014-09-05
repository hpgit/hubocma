#include "hubomaincontroller.h"
#include "huboglviewer.h"
#include <QApplication>
#include <HuboVpController.h>
#include <CmaOptimizer.h>
#include <UniformBspline.h>

HuboVpController *hubo, *huboRefer;
//HuboGlWindow *huboGl, *huboReferGl;
CmaOptimizer cma;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    time_t start = time(NULL);

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
//win->end();
//
///*
//HuboTrackingViewer *contWin = new HuboTrackingViewer(1100, 200, 1024, 768, "Hubo with Controller");
//contWin->init(hubo);
//contWin->end();
//
//contWin->setReferMotion(huboRefer->huboVpBody->pHuboMotion);
////contWin->cmaRun();
////contWin->cma.loadSolution("../CmaData/trackingCmaSolution.txt");
////contWin->setCmaMotion();
//contWin->setInitMotion();
//
//contWin->show();
////*/
    win.show();
//
////Fl_Window *form2 = new Fl_Window(1024, 768, "HuboHubo");
//HuboGlWindow *ggg = new HuboGlWindow(0,0,300, 200);
//ggg->cont = huboRefer;
//ggg->pHuboMotion = huboRefer->huboVpBody->pHuboMotion;
//ggg->end();
//ggg->show();
////form2->end();
////form2->show();
//std::cout<< time(NULL) - start <<std::endl;


    HuboMainController w;

    w.show();

    return a.exec();
}
