#include "huboglwidget.h"

HuboGlWidget::HuboGlWidget(QWidget *parent)
    : MotionGlWidget(parent)
{
    pHuboMotion = NULL;
    isAutoRepeat = 1;
    playing = 0;
}

void HuboGlWidget::paintGL()
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
    if(pHuboMotion != NULL)
    {
        int frame = pHuboMotion->getCurrentFrame();
        double frameTime = pHuboMotion->getFrameTime();
        //pHuboMotion->draw(frame);
		pHuboMotion->drawBox(frame);

        glDisable(GL_LIGHTING);
        glPointSize(5.0);
        glBegin(GL_POINTS);
                glColor3f(1,0,0);
                Vector3d com = pHuboMotion->getHuboComGlobalPosition();
                glVertex3dv(com.data());
                com.y() = 0;
                glVertex3dv(com.data());
				glColor3f(0,0,1);
				glVertex3dv(pHuboMotion->getFootCenter().data());
                //glVertex3dv(cont->huboOdeBody->getHuboCom().data());
        glEnd();
        //if (cont != NULL)
        //      cont->huboVpBody->drawAllBoundingBox();

        glPointSize(1.0);
    }
}

int HuboGlWidget::getFrame()
{
        return pHuboMotion->getCurrentFrame();
}

void HuboGlWidget::setFrame(int _frame)
{
        pHuboMotion->setCurrentFrame(_frame);
}

void HuboGlWidget::goOneFrame()
{
        int frame = pHuboMotion->getCurrentFrame();
        if(frame+1 >= pHuboMotion->getMotionSize())
        {
                if(isAutoRepeat == 1)
                        frame = -1;
                else
                        frame--;
        }
        pHuboMotion->setCurrentFrame(frame+1);
}

void HuboGlWidget::addOneFrame()
{
        pHuboMotion->addMotionSize(1);
}
