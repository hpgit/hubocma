#ifndef HUBOREFERVIEWER_H
#define HUBOREFERVIEWER_H

#include "huboglviewer.h"

class HuboReferViewer : public HuboGlViewer
{
public:
    HuboReferViewer(QWidget *parent = 0);

	void setReferMotion(HuboMotionData *_refer);
	HuboMotionData *refer;
	//void rFoot(Eigen::Vector3d &pos, Eigen::Quaterniond &ori);
	void rFoot();
	void solve(std::string name, Eigen::Vector3d &dpos, bool parallel,
		int maxIter, double ikEps, double weightPos, double weightAng, double stepSize);
};

#endif // HUBOREFERVIEWER_H
