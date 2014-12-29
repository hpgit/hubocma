#include "huboikmanage.h"
#include "ui_huboikmanage.h"

HuboIkManage::HuboIkManage(QWidget *parent)
	: QDialog(parent)
{
	ui = new Ui::HuboIkManage();
	ui->setupUi(this);
}

HuboIkManage::~HuboIkManage()
{
	delete ui;
}

void HuboIkManage::init(HuboIkViewer *_viewer)
{
	viewer = _viewer;
	
	ui->textEdit->setText(QString("RAR"));
	ui->maxIterEdit->setText(QString("400"));
	ui->ikEpsEdit->setText(QString("0.00001"));
	ui->weightAngEdit->setText(QString("10"));
	ui->weightPosEdit->setText(QString("1"));
	ui->stepSizeEdit->setText(QString("0.001"));
	
	ui->dxEdit->setText(QString("0.0"));
	ui->dyEdit->setText(QString("0.0"));
	ui->dzEdit->setText(QString("0.0"));

	ui->beginFrameEdit->setText(QString("0"));
	ui->endFrameEdit->setText(QString("0"));
}

void HuboIkManage::on_solveBtn_clicked()
{
	Eigen::Vector3d v;
	v.x() = ui->dxEdit->toPlainText().toDouble();
	v.y() = ui->dyEdit->toPlainText().toDouble();
	v.z() = ui->dzEdit->toPlainText().toDouble();
	viewer->solve(
		ui->textEdit->toPlainText().toStdString(),
		v,
		ui->parallelCheck->isChecked(),
		ui->maxIterEdit->toPlainText().toInt(),
		ui->ikEpsEdit->toPlainText().toDouble(),
		ui->weightPosEdit->toPlainText().toDouble(),
		ui->weightAngEdit->toPlainText().toDouble(),
		ui->stepSizeEdit->toPlainText().toDouble()
		);
}

void HuboIkManage::on_solveAllBtn_clicked()
{
	int _frame = viewer->refer->getCurrentFrame();

	Eigen::Vector3d v;
	v.x() = ui->dxEdit->toPlainText().toDouble();
	v.y() = ui->dyEdit->toPlainText().toDouble();
	v.z() = ui->dzEdit->toPlainText().toDouble();
	for(int i=0; i<viewer->refer->getMotionSize(); i++)
	{
		viewer->solve(
					ui->textEdit->toPlainText().toStdString(),
					v,
					ui->parallelCheck->isChecked(),
					ui->maxIterEdit->toPlainText().toInt(),
					ui->ikEpsEdit->toPlainText().toDouble(),
					ui->weightPosEdit->toPlainText().toDouble(),
					ui->weightAngEdit->toPlainText().toDouble(),
					ui->stepSizeEdit->toPlainText().toDouble()
					);
		if (viewer->refer->canGoOneFrame())
			viewer->refer->setCurrentFrame(viewer->refer->getCurrentFrame() + 1);
	}

	viewer->refer->setCurrentFrame(_frame);
}


void HuboIkManage::on_backupBtn_clicked()
{
	HuboMotionData *data = viewer->hubo->getHuboMotion();
	data->backUpMotion(data->getCurrentFrame());
	viewer->glWidget->updateGL();
}

void HuboIkManage::on_initBtn_clicked()
{
	HuboMotionData *data = viewer->hubo->getHuboMotion();
	data->copyOneMotion(viewer->refer, data->getCurrentFrame(), data->getCurrentFrame());
	viewer->glWidget->updateGL();
}

void HuboIkManage::on_initAllBtn_clicked()
{
	HuboMotionData *data = viewer->hubo->getHuboMotion();
	data->copyAllMotion(viewer->refer);
	viewer->glWidget->updateGL();
}

void HuboIkManage::on_propagateBtn_clicked()
{
	HuboMotionData *data = viewer->hubo->getHuboMotion();
	int begin = ui->beginFrameEdit->toPlainText().toInt();
	int end = ui->endFrameEdit->toPlainText().toInt();

	data->propagateDiffer(begin, end);
}

void HuboIkManage::on_saveBtn_clicked()
{
	HuboMotionData *data = viewer->hubo->getHuboMotion();
	QString filename = QFileDialog::getSaveFileName();
	data->save(filename.toStdString().data(), 0);
}

void HuboIkManage::on_applyBtn_clicked()
{
	HuboMotionData *data = viewer->hubo->getHuboMotion();
	viewer->refer->copyAllMotion(data);
}
