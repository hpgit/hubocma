#ifndef HUBOREFERVIEWER_H
#define HUBOREFERVIEWER_H

#include "huboglviewer.h"

class HuboReferViewer : public HuboGlViewer
{
public:
    HuboReferViewer(QWidget *parent = 0);

	void setReferMotion(HuboMotionData *_refer);
	HuboMotionData *refer;
};

#endif // HUBOREFERVIEWER_H
