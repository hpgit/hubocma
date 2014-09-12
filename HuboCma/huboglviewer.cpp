#include "huboglviewer.h"
#include "ui_huboglviewer.h"

#include <QScrollArea>
#include <iostream>
#include <QTimer>

HuboGlViewer::HuboGlViewer(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HuboGlViewer)
{
    ui->setupUi(this);

}

HuboGlViewer::~HuboGlViewer()
{
    delete ui;
}

void HuboGlViewer::init(HuboVpController *_hubo)
{
    hubo = _hubo;
    glWidget = new HuboGlWidget(this);
    glWidget->pHuboMotion = _hubo->huboVpBody->pHuboMotion;
    glWidget->pHuboMotion->setOBJon();

    this->resize(600, 600);

    ui->glLayout->addWidget(glWidget, 0, 0);

    displayTimer = new QTimer(this);
	displayTimer->setInterval(glWidget->pHuboMotion->getFrameTime());
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(timer()));

    ui->frameSlider->setRange(0, _hubo->huboVpBody->pHuboMotion->getMotionSize()-1);
    ui->frameSlider->setValue(0);
    ui->frameSlider->setSingleStep(1);

    playing = 0;


}

void HuboGlViewer::resizeEvent(QResizeEvent *)
{
    QPoint laypos = ui->gridLayoutWidget->pos();
    ui->gridLayoutWidget->resize(width()-2*laypos.x(), height() - ui->frameSlider->height() - 2*laypos.y() );
    glWidget->resize(width(), height()-ui->playBtn->height());

    ui->playBtn->move(0,height()-ui->playBtn->height());
	
    ui->frameSlider->move(ui->playBtn->width(), height()-ui->frameSlider->height());
    ui->frameSlider->resize(width() - ui->playBtn->width(), ui->frameSlider->height());

	ui->textEdit->move(ui->playBtn->width(), height() - ui->frameSlider->height());
    ui->textEdit->resize(ui->textEdit->width(), ui->frameSlider->height());

    ui->frameSlider->move(ui->playBtn->width() + ui->textEdit->width(), height()-ui->frameSlider->height());
    ui->frameSlider->resize(width() - ui->playBtn->width() - ui->textEdit->width(), ui->frameSlider->height());
}

void HuboGlViewer::timer()
{
    glWidget->goOneFrame();
    ui->frameSlider->setValue(glWidget->pHuboMotion->getCurrentFrame());
	char buf[32];
	sprintf(buf, "%d", ui->frameSlider->value());
	ui->textEdit->setText(QString(buf));
    glWidget->updateGL();
}

void HuboGlViewer::adjustHuboMotionToViewer()
{
	displayTimer->stop();
	displayTimer->setInterval(glWidget->pHuboMotion->getFrameTime());
    ui->frameSlider->setRange(0, hubo->huboVpBody->pHuboMotion->getMotionSize()-1);
    ui->frameSlider->setValue(glWidget->pHuboMotion->getCurrentFrame());
	char buf[32];
	sprintf(buf, "%d", ui->frameSlider->value());
	ui->textEdit->setText(QString(buf));
}

/*
QSlider* HUBOViewDialog::createSlider(const char *changedSignal, const char *setterSlot)
{
    QSlider *slider = new QSlider(Qt::Horizontal);
    slider->setRange(0, 360 * 16);
    slider->setSingleStep(16);
    slider->setPageStep(15 * 16);
    slider->setTickInterval(15 * 16);
    slider->setTickPosition(QSlider::TicksRight);
    connect(slider, SIGNAL(valueChanged(int)), glWidget, setterSlot);
    connect(glWidget, changedSignal, slider, SLOT(setValue(int)));
    return slider;
}
*/

void HuboGlViewer::on_playBtn_clicked()
{
	if (displayTimer->isActive())
		displayTimer->stop();
	else
		displayTimer->start();
        //displayTimer->start(100);
}

void HuboGlViewer::on_frameSlider_valueChanged(int value)
{
	char buf[32];
	sprintf(buf, "%d", value);
	ui->textEdit->setText(QString(buf));
    glWidget->pHuboMotion->setCurrentFrame(value);
    glWidget->updateGL();
}
