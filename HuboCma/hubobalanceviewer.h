#ifndef HUBOBALANCEVIEWER_H
#define HUBOBALANCEVIEWER_H

#include "huboglviewer.h"

class HuboBalanceViewer : public HuboGlViewer
{
public:
	HuboBalanceViewer(QWidget *parent = 0);

	void setCmaMotion(
		double kl, double kh,
		double weightTrack, double weightTrackAnkle, double weightTrackUpper,
		int frameRate = 0
			);
	void setReferMotion(HuboMotionData *refer);

};

#endif // HUBOBALANCEVIEWER_H
