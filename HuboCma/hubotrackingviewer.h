#ifndef HUBOTRACKINGVIEWER_H
#define HUBOTRACKINGVIEWER_H

#include "huboglviewer.h"
#include <CmaOptimizer.h>

class HuboTrackingViewer : public HuboGlViewer
{
public:
    HuboTrackingViewer(QWidget *parent = 0);

    CmaOptimizer cma;
	std::vector<double> manualSol;
    void useLatestCmaResult(char* filename, std::vector<double> &solution);
    void cmaRun(int maxIter = 1000, int useLatestResult = 0);
	void setCmaMotion(int frameRate = 0, int useManualSolution = 0);
    void setReferMotion(HuboMotionData *refer);
    void setInitMotion();
};

#endif // HUBOTRACKINGVIEWER_H
