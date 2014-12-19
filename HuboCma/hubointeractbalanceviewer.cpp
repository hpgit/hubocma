#include "hubointeractbalanceviewer.h"
#include "ui_hubointeractbalanceviewer.h"

#include <QTimer>
#include "hubointeractbalancemanage.h"

static HuboInteractBalanceManage *_manager;

HuboInteractBalanceViewer::HuboInteractBalanceViewer(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::HuboInteractBalanceViewer)
{
	ui->setupUi(this);

	HuboInteractBalanceManage *manager = new HuboInteractBalanceManage();
	manager->viewer = this;
	manager->move(200+width()+20, 200);
	manager->show();
	_manager = manager;
}

HuboInteractBalanceViewer::~HuboInteractBalanceViewer()
{
	delete ui;
}

void HuboInteractBalanceViewer::initCont(HuboVpController *_hubo)
{
	hubo = _hubo;
	glWidget = new HuboVpGlWidget(this);

	glWidget->cont = _hubo;
	//glWidget->pHuboMotion = _hubo->huboVpBody->pHuboMotion;
	glWidget->cont->huboVpBody->pHuboMotion->setOBJon();

	_manager->initTimer();

	this->resize(600, 600);

	ui->glLayout->addWidget(glWidget, 0, 0);

	playing = 0;
}

void HuboInteractBalanceViewer::resizeEvent(QResizeEvent *)
{
	QPoint laypos = ui->gridLayoutWidget->pos();
	ui->gridLayoutWidget->resize(width()-2*laypos.x(), height() - ui->playBtn->height() - 2*laypos.y() );
	glWidget->resize(width(), height()-ui->playBtn->height());

	ui->playBtn->move(0,height()-ui->playBtn->height());

	//ui->frameSlider->move(ui->playBtn->width(), height()-ui->frameSlider->height());
	//ui->frameSlider->resize(width() - ui->playBtn->width(), ui->frameSlider->height());

	ui->timeLabel->move(ui->playBtn->width(), height() - ui->playBtn->height());
	ui->timeLabel->resize(ui->timeLabel->width(), ui->timeLabel->height());

	//ui->frameSlider->move(ui->playBtn->width() + ui->textEdit->width(), height()-ui->frameSlider->height());
	//ui->frameSlider->resize(width() - ui->playBtn->width() - ui->textEdit->width(), ui->frameSlider->height());
}

void HuboInteractBalanceViewer::setReferMotion(HuboMotionData *_refer)
{
	refer = _refer;
}
