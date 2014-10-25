#include "motionglwidget.h"
#include <QtGui>
#include <iostream>
#ifdef WIN32
#include <Windows.h>
#endif
#ifdef __APPLE__
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

MotionGlWidget::MotionGlWidget(QWidget *parent)
    : QGLWidget(parent)
{
}
MotionGlWidget::~MotionGlWidget()
{
    makeCurrent();
}

void MotionGlWidget::draw_ground()
{
        int count = 0;
        glBegin(GL_QUADS);
        for (int i = -80; i < 81; ++i) {
                for (int j = -80; j < 81; ++j) {
                        if (count % 2 == 0)
                                glColor3d(0.88, 0.88, 0.88);
                                //glColor3d(0.90, 0.90, 0.90);
                        else
                                glColor3d(0.93, 0.93, 0.93);
                                //glColor3d(0.95, 0.95, 0.95);

                        glNormal3d(0.,0., 1.);

                        glVertex3f(j, 0, i);
                        glVertex3f(j, 0, i+1);
                        glVertex3f(j+1, 0, i+1);
                        glVertex3f(j+1, 0, i);
                        count += 1;
                }
        }
        glEnd();
}

void MotionGlWidget::setupFloor()
{
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS,0x1,0x1);
        glStencilOp(GL_REPLACE,GL_REPLACE,GL_REPLACE);
        glStencilMask(0x1);             // only deal with the 1st bit
}

void MotionGlWidget::draw_axis()
{
        glBegin(GL_LINES);
        glColor3f(1,0,0);
        glVertex3f(1.0,0,0);
        glVertex3f(0,0,0);
        glColor3f(0,1,0);
        glVertex3f(0,1.0,0);
        glVertex3f(0,0,0);
        glColor3f(0,0,1);
        glVertex3f(0,0,1.0);
        glVertex3f(0,0,0);
        glEnd();
}

void MotionGlWidget::init_gl_color()
{
        GLfloat light_ambient[] =  {0.0, 0.0, 0.0, 1.0};
        GLfloat light_diffuse[] =  {1.0, 1.0, 1.0, 1.0};
        GLfloat light_specular[] =  {1.0, 1.0, 1.0, 1.0};
        GLfloat light_position[] =  {1.0, 1.0, 1.0, 0.0};

        GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
        GLfloat mat_shininess = 70.0;

        glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
        glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);

        GLfloat ambient[] = {0.7, 0.7, 0.7, 1};
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
        glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
        glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
        glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);

        //glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_COLOR_MATERIAL);

        glEnable(GL_NORMALIZE);

        glEnable(GL_POINT_SMOOTH);
}

void MotionGlWidget::initializeGL()
{
        init_gl_color();
        //glMatrixMode(GL_PROJECTION);
        //glLoadIdentity();
        //gluPerspective( 45., float(w())/float(h()), 0.1, 1000.);
        //glMatrixMode(GL_MODELVIEW);
        //glLoadIdentity();
}

void MotionGlWidget::paintGL()
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
}

void MotionGlWidget::_draw()
{
    // draw here
}

void MotionGlWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45., float(width)/float(height), 0.1, 1000.);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void MotionGlWidget::mousePressEvent(QMouseEvent *event)
{
    if(event->button() == Qt::MidButton || event->button() == Qt::RightButton)
    {
        lastPos = event->pos();
    }
}

void MotionGlWidget::mouseMoveEvent(QMouseEvent *event)
{
    currPos = event->pos();
    int dx = currPos.x() - lastPos.x();
    int dy = - currPos.y() + lastPos.y();

	if (event->buttons() & Qt::LeftButton)
    {
        camera.setPosOnSphereAfterScreenRotate(dx, dy, width(), height());
        camera.setTransforming();
        repaint();
    }
    else if (event->buttons() & Qt::RightButton)
    {
        camera.addPosAfterScreenShift(dx, dy, height(), M_PI/4.);
        camera.setTransforming();
        repaint();
    }
}

void MotionGlWidget::mouseReleaseEvent(QMouseEvent *event)
{
    camera.invalidate();
    camera.unsetTransforming();
    repaint();
}

void MotionGlWidget::wheelEvent(QWheelEvent *event)
{
    if(event->modifiers().testFlag(Qt::ControlModifier))
    {
        double dist = camera.distance;
        camera.setDistance((8.0+event->delta())*dist/8.0);
        //glTranslatef(-event->delta()/1000.0, 0, 0);
    }
}
