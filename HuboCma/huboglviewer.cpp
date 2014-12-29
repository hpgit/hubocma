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

void HuboGlViewer::initCont(HuboGearController *_hubo)
{
    hubo = _hubo;
    glWidget = new HuboGlWidget(this);
	glWidget->pHuboMotion = _hubo->getHuboMotion();
    glWidget->pHuboMotion->setOBJon();

    this->resize(600, 600);

    ui->glLayout->addWidget(glWidget, 0, 0);

    displayTimer = new QTimer(this);
	displayTimer->setInterval(glWidget->pHuboMotion->getFrameTime());
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(timer()));

	ui->frameSlider->setRange(0, _hubo->getHuboMotion()->getMotionSize()-1);
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
	ui->textEdit->setText(QString::number(ui->frameSlider->value()));
    glWidget->updateGL();
}

void HuboGlViewer::adjustHuboMotionToViewer()
{
	displayTimer->stop();
	displayTimer->setInterval(glWidget->pHuboMotion->getFrameTime());
	ui->frameSlider->setRange(0, hubo->getHuboMotion()->getMotionSize()-1);
    ui->frameSlider->setValue(glWidget->pHuboMotion->getCurrentFrame());
	ui->textEdit->setText(QString::number(ui->frameSlider->value()));
	glWidget->updateGL();
}

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
	ui->textEdit->setText(QString::number(ui->frameSlider->value()));
    glWidget->pHuboMotion->setCurrentFrame(value);
    glWidget->updateGL();
}
