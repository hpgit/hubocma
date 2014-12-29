#include "hubovpglwidget.h"

HuboVpGlWidget::HuboVpGlWidget(QWidget *parent)
	: MotionGlWidget(parent)
{
	cont = NULL;
	playing = 0;
	forceDraw = 0;
}

void HuboVpGlWidget::forceDrawOn(Eigen::Vector3d &_force, Eigen::Vector3d &_pos)
{
	force = _force;
	forcePos = _pos;
	forceDraw = 1;
}

void HuboVpGlWidget::forceDrawOff()
{
	forceDraw = 0;
}

void HuboVpGlWidget::paintGL()
{
	glClearColor(1,1,1,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	camera.camera_lookat();

	glDisable(GL_LIGHTING);
	setupFloor();
	draw_ground();
	draw_axis();

	glColor3d(0., 0., 0.);

	glEnable(GL_LIGHTING);
	if(cont != NULL)
	{
		HuboMotionData *pHuboMotion = cont->getHuboMotion();
		cont->huboGearBody->applyAllJointValueVptoHubo();
		//pHuboMotion->draw(frame);
		pHuboMotion->drawBox();
		if(forceDraw)
		{
			Eigen::Vector3d startForcePos = forcePos - force/50.0;
			glLineWidth(5.0);
			glBegin(GL_LINES);
			glColor3f(0.2, 0.2, 0.8);
			glVertex3dv(forcePos.data());
			glVertex3dv(startForcePos.data());
			glEnd();
		}

		glDisable(GL_LIGHTING);
		glPointSize(5.0);
		glBegin(GL_POINTS);
			glColor3f(1,0,0);
			//Eigen::Vector3d com = pHuboMotion->getHuboComGlobalPosition();
			Eigen::Vector3d com = cont->huboGearBody->getCOMposition();
			glVertex3dv(com.data());
			//com.y() = 0;
			glVertex3dv(com.data());
			glColor3f(0,0,1);
			//glVertex3dv(pHuboMotion->getFootCenter().data());
			glVertex3dv(cont->huboGearBody->getSupportRegionCenter().data());
		glEnd();


		//if (cont != NULL)
		//      cont->huboVpBody->drawAllBoundingBox();

		glPointSize(1.0);
	}
}
