#ifndef HUBOINTERACTBALANCEVIEWER_H
#define HUBOINTERACTBALANCEVIEWER_H

#include <QDialog>
#include <QGLWidget>
#include <QScrollArea>
#include "hubovpglwidget.h"

namespace Ui {
class HuboInteractBalanceViewer;
}

class HuboInteractBalanceViewer : public QDialog
{
	Q_OBJECT

public:
	explicit HuboInteractBalanceViewer(QWidget *parent = 0);
	~HuboInteractBalanceViewer();

	void initCont(HuboGearController *_hubo);

	void resizeEvent(QResizeEvent *);
	void adjustHuboMotionToViewer();

	QTimer			*displayTimer;
	HuboVpGlWidget	*glWidget;
	HuboGearController *hubo;
	HuboMotionData *refer;
	int playing;
	void setReferMotion(HuboMotionData *refer);


private:
	Ui::HuboInteractBalanceViewer *ui;
};

#endif // HUBOINTERACTBALANCEVIEWER_H
