#include "hubocmamanage.h"
#include "ui_hubocmamanage.h"

HuboCmaManage::HuboCmaManage(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HuboCmaManage)
{
    ui->setupUi(this);
	cb.cmaman = this;
	cb.show();
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

	QStringList list=
		(QStringList()
		<< "LHY"
		<< "LHR"
		<< "LHP"
		<< "LKN"
		<< "LAP"
		<< "LAR"
		<< "RHY"
		<< "RHR"
		<< "RHP"
		<< "RKN"
		<< "RAP"
		<< "RAR"
		);
	ui->plotComboBox->addItems(list);

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
	viewer->cma.loadSolution("../../dat/CmaData/trackingCmaSolution.txt");
	//ui->simulateBtn->setEnabled(true);
	//ui->runCmaWithResBtn->setEnabled(true);
}

void HuboCmaManage::on_saveBtn_clicked()
{
	viewer->cma.saveSolution("../../dat/CmaData/trackingCmaSolution.txt");
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

void HuboCmaManage::on_plotBtn_clicked()
{
	QString str = ui->plotComboBox->currentText();
	int num = 0;
	if (!str.compare("LHY")) num = 0;
	else if (!str.compare("LHR")) num = 1;
	else if (!str.compare("LHP")) num = 2;
	else if (!str.compare("LKN")) num = 3;
	else if (!str.compare("LAP")) num = 4;
	else if (!str.compare("LAR")) num = 5;
	else if (!str.compare("RHY")) num = 6;
	else if (!str.compare("RHR")) num = 7;
	else if (!str.compare("RHP")) num = 8;
	else if (!str.compare("RKN")) num = 9;
	else if (!str.compare("RAP")) num = 10;
	else if (!str.compare("RAR")) num = 11;

	if (viewer->cma.hasSolution == 1)
		cb.setBezierSpline(viewer->cma.solution, num);

}

void HuboCmaManage::physSetting()
{
	viewer->hubo->grfKs = ui->grfKsEdit->toPlainText().toDouble();
	viewer->hubo->grfDs = ui->grfKdEdit->toPlainText().toDouble();
	viewer->hubo->ks = ui->PDKsEdit->toPlainText().toDouble();
	viewer->hubo->kd = ui->PDKdEdit->toPlainText().toDouble();
}

void HuboCmaBezier::setBezierSpline(std::vector<double> &sol, int num)
{
	Eigen::VectorXd v;
	v.resize(1);
	bs.clear();
	for (int i = 0; i < 4; i++)
	{
		int k = 12 * (i % 3) + num;
		v(0) = sol.at(12 * (i%3) + num);
		bs.setControlPoint(i, v);
	}
	repaint();
}

void HuboCmaBezier::paintEvent(QPaintEvent *event)
{
	const int halfLineLength = 10;
	double rad;
	QPainter p(this);

	//y axis
	p.drawLine(30, 60, 30, 470);

	//x axis
	p.drawLine(25, 240, 680, 240);

	// x=1 line
	p.drawLine(530, 240+halfLineLength, 530, 240-halfLineLength);

	// y=-pi/2 line
	p.drawLine(30 - halfLineLength, 390, 30 + halfLineLength, 390);

	// y=pi/2 line
	p.drawLine(30 - halfLineLength, 90, 30 + halfLineLength, 90);

	QColor red(255, 0, 0);
	QColor black(0, 0, 0);

	for (int i = 0; i <= 500; i++)
	{
		double value = bs.getValue((double)i / 500)(0);
		if (i % 125 == 0)
		{
			QString str = QString::number(value / M_PI) + QString("pi");
			p.setPen(black);
			p.drawText(i + 30, -value * 300 / M_PI+240+20, str);
		}
		p.setPen(red);
		p.drawPoint(
			i + 30, 
			-value*300/M_PI + 240
		);
	}
}
