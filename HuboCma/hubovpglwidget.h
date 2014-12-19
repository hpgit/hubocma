#ifndef HUBOVPGLWIDGET_H
#define HUBOVPGLWIDGET_H

#include <motionglwidget.h>
#include <HuboVpController.h>

class HuboVpGlWidget : public MotionGlWidget
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	HuboVpGlWidget(QWidget *parent = 0);
	HuboVpController *cont;

	int playing;
	int forceDraw;
	Eigen::Vector3d force;
	Eigen::Vector3d forcePos;

	void forceDrawOn(Eigen::Vector3d &force, Eigen::Vector3d &pos);
	void forceDrawOff();

protected:
	void paintGL();
};

#endif // HUBOVPGLWIDGET_H
