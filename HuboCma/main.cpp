#include "hubomaincontroller.h"
#include "hubotrackingviewer.h"
#include "huboikviewer.h"
#include <QApplication>
//#include <HuboVpController.h>
#include <CmaOptimizer.h>
#include <UniformBspline.h>
#ifdef WIN32
#include <ctime>
#endif

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

	HuboMainController *mainCont = new HuboMainController;
	mainCont->setWindowTitle(QString("Hubo Main Controller"));
	mainCont->show();

    return a.exec();
}
