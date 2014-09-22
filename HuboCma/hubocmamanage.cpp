#include "hubocmamanage.h"
#include "ui_hubocmamanage.h"

HuboCmaManage::HuboCmaManage(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HuboCmaManage)
{
    ui->setupUi(this);
}

HuboCmaManage::~HuboCmaManage()
{
    delete ui;
}

void HuboCmaManage::initManager(HuboTrackingViewer *_viewer)
{
	viewer = _viewer;
	ui->iterEdit->setText(QString("1000"));
	ui->grfKsEdit->setText(QString::number(viewer->hubo->grfKs));
	ui->grfKdEdit->setText(QString::number(viewer->hubo->grfDs));
	ui->PDKsEdit->setText( QString::number(viewer->hubo->ks));
	ui->PDKdEdit->setText( QString::number(viewer->hubo->kd));

	/*
	ui->simulateBtn->setDisabled(true);
	ui->runCmaWithResBtn->setDisabled(true);
	ui->resumeBtn->setDisabled(true);
	ui->pauseBtn->setDisabled(true);
	ui->stopBtn->setDisabled(true);
	*/
}

void HuboCmaManage::on_runCmaBtn_clicked()
{
	physSetting();
    viewer->cmaRun(ui->iterEdit->toPlainText().toInt(), 0);
    //viewer->setCmaMotion(0, 0);
	//ui->pauseBtn->setEnabled(true);
	//ui->stopBtn->setEnabled(true);
}

void HuboCmaManage::on_runCmaWithResBtn_clicked()
{
	physSetting();
    viewer->cmaRun(ui->iterEdit->toPlainText().toInt(), 1);
    //viewer->setCmaMotion(0, 0);
	//ui->pauseBtn->setEnabled(true);
	//ui->stopBtn->setEnabled(true);
}

void HuboCmaManage::on_loadBtn_clicked()
{
	viewer->cma.loadSolution("../CmaData/trackingCmaSolution.txt");
	//ui->simulateBtn->setEnabled(true);
	//ui->runCmaWithResBtn->setEnabled(true);
}

void HuboCmaManage::on_saveBtn_clicked()
{
	viewer->cma.saveSolution("../CmaData/trackingCmaSolution.txt");
}

void HuboCmaManage::on_pauseBtn_clicked()
{
	viewer->cma.pause();
	//ui->resumeBtn->setEnabled(true);
	//ui->pauseBtn->setDisabled(true);
}

void HuboCmaManage::on_resumeBtn_clicked()
{
	viewer->cma.resume();
	//ui->resumeBtn->setDisabled(true);
	//ui->pauseBtn->setEnabled(true);
}

void HuboCmaManage::on_stopBtn_clicked()
{
	viewer->cma.stop();
	viewer->cmaTh.wait();
	//ui->stopBtn->setDisabled(true);
	//ui->pauseBtn->setDisabled(true);
}

void HuboCmaManage::on_simulateBtn_clicked()
{
	physSetting();
	viewer->setCmaMotion(0, 1);
}

void HuboCmaManage::physSetting()
{
	viewer->hubo->grfKs = ui->grfKsEdit->toPlainText().toDouble();
	viewer->hubo->grfDs = ui->grfKdEdit->toPlainText().toDouble();
	viewer->hubo->ks = ui->PDKsEdit->toPlainText().toDouble();
	viewer->hubo->kd = ui->PDKdEdit->toPlainText().toDouble();
}