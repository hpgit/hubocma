#include <QtWidgets>
#include "huborefermanage.h"
#include "ui_huborefermanage.h"

HuboReferManage::HuboReferManage(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::HuboReferManage)
{
	ui->setupUi(this);
}

HuboReferManage::~HuboReferManage()
{
	delete ui;
}

void HuboReferManage::on_saveBtn_clicked()
{
	HuboMotionData *data = viewer->hubo->getHuboMotion();
	QString filename = QFileDialog::getSaveFileName();
	data->save(filename.toStdString().data(), 0);
}

void HuboReferManage::on_cutFrontBtn_clicked()
{
	HuboMotionData *data = viewer->hubo->getHuboMotion();
	data->cutMotion(data->getCurrentFrame());
	viewer->adjustHuboMotionToViewer();
}

void HuboReferManage::on_cutBackBtn_clicked()
{
	HuboMotionData *data = viewer->hubo->getHuboMotion();
	data->cutMotion(0, data->getCurrentFrame());
	viewer->adjustHuboMotionToViewer();
}
