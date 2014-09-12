#include "hubotrackingmanage.h"
#include "ui_hubotrackingmanage.h"

HuboTrackingManage::HuboTrackingManage(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HuboTrackingManage)
{
    ui->setupUi(this);
}

HuboTrackingManage::~HuboTrackingManage()
{
    delete ui;
}

void HuboTrackingManage::init(HuboTrackingViewer *_viewer)
{
	viewer = _viewer;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 12; j++)
		{
			viewer->manualSol.push_back(0);
			sliders.push_back(new QSlider(Qt::Horizontal));
			sliders.back()->setRange(0, 1000);
			sliders.back()->setValue(500);
			sliders.back()->setSingleStep(1);
			sliders.back()->setTickInterval(20);
			if (j < 6)
				ui->gridLayout->addWidget(sliders.back(), j, 2 * i);
			else
				ui->gridLayout_2->addWidget(sliders.back(), j - 6, 2 * i);

			values.push_back(new QTextEdit(QString("0")));
			values.back()->setMaximumSize(45, 30);
			if (j < 6)
				ui->gridLayout->addWidget(values.back(), j, 2*i+1);
			else
				ui->gridLayout_2->addWidget(values.back(), j-6, 2*i+1);
		}
	}
	ui->groundKs->setText(QString(QString::number(viewer->hubo->grfKs)));
	ui->groundKd->setText(QString(QString::number(viewer->hubo->grfDs)));
	ui->servoKs->setText(QString(QString::number(viewer->hubo->ks)));
	ui->servoKd->setText(QString(QString::number(viewer->hubo->kd)));

	for (int i = 0; i < sliders.size(); i++)
		connect(sliders.at(i), SIGNAL(valueChanged(int)), this, SLOT(slider_valueChanged()));
}

void HuboTrackingManage::on_simulBtn_clicked()
{
	viewer->setCmaMotion(0, 1);
}

void HuboTrackingManage::on_runCmaWithThis_clicked()
{

}

void HuboTrackingManage::on_runCmaWithRes_clicked()
{

}
void HuboTrackingManage::on_groundSetBtn_clicked()
{
	viewer->hubo->grfKs = ui->groundKs->toPlainText().toDouble();
	viewer->hubo->grfDs = ui->groundKd->toPlainText().toDouble();
}

void HuboTrackingManage::on_servoSetBtn_clicked()
{
	viewer->hubo->ks = ui->servoKs->toPlainText().toDouble();
	viewer->hubo->kd = ui->servoKd->toPlainText().toDouble();
}

void HuboTrackingManage::slider_valueChanged()
{
	for (int i = 0; i < sliders.size(); i++)
	{
		char value[32];
		viewer->manualSol.at(i) = ((double)sliders.at(i)->value() - 500)*M_PI / 1000.;
		sprintf(value, "%.3lf", viewer->manualSol.at(i));
		values.at(i)->setText(value);
	}
	
}