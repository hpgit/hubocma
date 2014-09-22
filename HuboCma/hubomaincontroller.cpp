#include <QtWidgets>
#include "hubomaincontroller.h"
#include "ui_hubomaincontroller.h"
#include "huboglviewer.h"
#include "hubotrackingviewer.h"
#include "huboikviewer.h"


HuboMainController::HuboMainController(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::HuboMainController)
{
	huboRefer = 0;
	huboIk = 0;
	huboCma = 0;
    ui->setupUi(this);
}

HuboMainController::~HuboMainController()
{
    delete ui;
}

void HuboMainController::on_loadReferBtn_clicked()
{
	QString title("Motion File");
	QString dir("../CmaData/");
	QString filename = QFileDialog::getOpenFileName(this, title, dir);
	QString title1("Motion Contact Info File");
	QString dir1("../CmaData/");
	QString filename1 = QFileDialog::getOpenFileName(this, title1, dir1);
	if (filename.length() > 0)
	{
		huboRefer = new HuboVpController;
		huboRefer->initController();
		huboRefer->huboVpBody->pHuboMotion->import(filename.toStdString().data(), 0);
		huboRefer->huboVpBody->pHuboMotion->importContactPeriodAnnotation(filename1.toStdString().data(), 0);
		HuboGlViewer *win = new HuboGlViewer;
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
		huboIk->huboVpBody->pHuboMotion->copyAllMotion(huboRefer->huboVpBody->pHuboMotion);
		HuboIkViewer *win = new HuboIkViewer;
		win->setWindowTitle(QString("Hubo IK Viewer"));
		win->initCont(huboIk);
		win->setReferMotion(huboRefer->huboVpBody->pHuboMotion);
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
		win->setReferMotion(huboRefer->huboVpBody->pHuboMotion);
		win->show();
	}
}