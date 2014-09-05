#ifndef MOTIONGLWIDGET_H
#define MOTIONGLWIDGET_H

#include <QGLWidget>
#include "camera.h"

class MotionGlWidget : public QGLWidget
{
    Q_OBJECT
public:
    //friend class HUBOViewDialog;

    MotionGlWidget(QWidget *parent = 0);
    ~MotionGlWidget();

    Camera camera;
    void _draw();

    QPoint currPos, lastPos;

protected:
    void setupFloor();
    void draw_ground();
    void draw_axis();
    void init_gl_color();

    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);
};

#endif // MOTIONGLWIDGET_H
