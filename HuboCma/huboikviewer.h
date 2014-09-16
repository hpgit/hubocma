#ifndef HUBOIKVIEWER_H
#define HUBOIKVIEWER_H

#include "huboglviewer.h"

class HuboIkViewer : public HuboGlViewer
{
public:
    HuboIkViewer(QWidget *parent = 0);

	//void rFoot(Eigen::Vector3d &pos, Eigen::Quaterniond &ori);
	void rFoot();
};

#endif // HUBOIKVIEWER_H
