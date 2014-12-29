#ifndef HUBOGLVIEWER_H
#define HUBOGLVIEWER_H

#include <QDialog>
#include <QGLWidget>
#include <QScrollArea>
#include "huboglwidget.h"

namespace Ui {
class HuboGlViewer;
}

class HuboGlViewer : public QDialog
{
    Q_OBJECT

public:
    explicit HuboGlViewer(QWidget *parent = 0);
    ~HuboGlViewer();

	void initCont(HuboGearController *_hubo);

    void resizeEvent(QResizeEvent *);
	void adjustHuboMotionToViewer();

    QTimer          *displayTimer;
    HuboGlWidget	*glWidget;
	HuboGearController *hubo;
    int playing;

private slots:
    void timer();

    void on_playBtn_clicked();

    void on_frameSlider_valueChanged(int value);

private:
    Ui::HuboGlViewer *ui;
};

#endif // HUBOGLVIEWER_H
