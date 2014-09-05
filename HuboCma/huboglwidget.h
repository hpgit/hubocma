#ifndef HUBOGLWIDGET_H
#define HUBOGLWIDGET_H

#include <motionglwidget.h>
#include <HuboMotionData.h>
#include <HuboVpController.h>

class HuboGlWidget : public MotionGlWidget
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    HuboGlWidget(QWidget *parent = 0);
    HuboMotionData *pHuboMotion;
    HuboVpController *cont;
    int isAutoRepeat;
    int playing;


    int getFrame();
    void setFrame(int _frame);
    void goOneFrame();
    void addOneFrame();
protected:
    void paintGL();
};

#endif // HUBOGLWIDGET_H
