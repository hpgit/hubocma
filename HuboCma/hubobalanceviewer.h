#ifndef HUBOBALANCEVIEWER_H
#define HUBOBALANCEVIEWER_H

#include "huboglviewer.h"

class HuboBalanceViewer : public HuboGlViewer
{
public:
	HuboBalanceViewer(QWidget *parent = 0);

	void setCmaMotion(int frameRate = 0);
	void setReferMotion(HuboMotionData *refer);

};

#endif // HUBOBALANCEVIEWER_H
