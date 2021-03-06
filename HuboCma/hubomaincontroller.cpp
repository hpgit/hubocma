#include <QtWidgets>
#include "hubomaincontroller.h"
#include "ui_hubomaincontroller.h"
#include "huboreferviewer.h"
#include "hubotrackingviewer.h"
#include "huboikviewer.h"
#include "hubobalanceviewer.h"
#include "hubointeractbalanceviewer.h"


HuboMainController::HuboMainController(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::HuboMainController)
{
	huboRefer = 0;
	huboIk = 0;
	huboCma = 0;
	huboBalance = 0;
	huboInterBalance = 0;
    ui->setupUi(this);
}

HuboMainController::~HuboMainController()
{
    delete ui;
}

void HuboMainController::on_loadReferBtn_clicked()
{
	QString title("Motion File");
	QString dir("../../dat/motiondata/");
	QString filename = QFileDialog::getOpenFileName(this, title, dir);
	//QString title1("Motion Contact Info File");
	//QString dir1("../../dat/motiondata/");
	//QString filename1 = QFileDialog::getOpenFileName(this, title1, dir1);
	if (filename.length() > 0)
	{
		huboRefer = new HuboVpController;
		huboRefer->initController();
		huboRefer->getHuboMotion()->import(filename.toStdString().data(), 0);
		//if(filename1.length() > 0)
		//	huboRefer->huboVpBody->pHuboMotion->importContactPeriodAnnotation(filename1.toStdString().data(), 0);
		HuboReferViewer *win = new HuboReferViewer;
		win->setWindowTitle(QString("Hubo Reference Motion"));
		win->initCont(huboRefer);
		win->show();
	}
}

void HuboMainController::on_ikDlgBtn_clicked()
{
	if (huboRefer != 0)
	{
		huboIk = new HuboVpController;
		huboIk->initController();
		huboIk->getHuboMotion()->copyAllMotion(huboRefer->getHuboMotion());
		HuboIkViewer *win = new HuboIkViewer;
		win->setWindowTitle(QString("Hubo IK Viewer"));
		win->initCont(huboIk);
		win->setReferMotion(huboRefer->getHuboMotion());
		win->show();
	}
}

void HuboMainController::on_cmaDlgBtn_clicked()
{
	if (huboRefer != 0)
	{
		huboCma = new HuboVpController;
		huboCma->initController();
		HuboTrackingViewer *win = new HuboTrackingViewer;
		win->setWindowTitle(QString("Hubo Tracking Viewer"));
		win->initCont(huboCma);
		win->setReferMotion(huboRefer->getHuboMotion());
		win->show();
	}
}

void HuboMainController::on_balanceDlgBtn_clicked()
{
	if (huboRefer != 0)
	{
		huboBalance = new HuboVpController;
		huboBalance->initController();
		HuboBalanceViewer *win = new HuboBalanceViewer;
		win->setWindowTitle(QString("Hubo Balance Viewer"));
		win->initCont(huboBalance);
		win->setReferMotion(huboRefer->getHuboMotion());
		win->show();
	}
}
void HuboMainController::on_interBalanceDlgBtn_clicked()
{
	if (huboRefer != 0)
	{
		huboInterBalance = new HuboVpController;
		huboInterBalance->initController();
		huboInterBalance->getHuboMotion()->setMotionSize(1);
		HuboInteractBalanceViewer *win = new HuboInteractBalanceViewer;
		win->setWindowTitle(QString("Hubo Interactive Balance Viewer"));
		win->initCont(huboInterBalance);
		win->setReferMotion(huboRefer->getHuboMotion());
		win->show();
	}
}
