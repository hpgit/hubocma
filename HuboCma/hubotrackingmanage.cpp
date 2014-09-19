#include "hubotrackingmanage.h"
#include "ui_hubotrackingmanage.h"

HuboTrackingManage::HuboTrackingManage(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::HuboTrackingManage)
{
    ui->setupUi(this);
	sliderMoveFlag = 0;
    textMoveFlag = 0;
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

			values.push_back(new QTextEdit(QString("0.000")));
			values.back()->setMaximumSize(45, 30);
			if (j < 6)
				ui->gridLayout->addWidget(values.back(), j, 2*i+1);
			else
				ui->gridLayout_2->addWidget(values.back(), j-6, 2*i+1);
		}
	}
	ui->groundKs->setText(QString::number(viewer->hubo->grfKs));
	ui->groundKd->setText(QString::number(viewer->hubo->grfDs));
	ui->servoKs->setText( QString::number(viewer->hubo->ks));
	ui->servoKd->setText( QString::number(viewer->hubo->kd));

	ui->cmaRunIterEdit->setText(QString("1000"));

	for (int i = 0; i < sliders.size(); i++)
		connect(sliders.at(i), SIGNAL(valueChanged(int)), this, SLOT(slider_valueChanged()));
    for (int i = 0; i < values.size(); i++)
        connect(values.at(i), SIGNAL(textChanged()), this, SLOT(text_valueChanged()));
}


void HuboTrackingManage::save(char *filename)
{
	std::ofstream fout;
	fout.open(filename);

	fout << ui->groundKs->toPlainText().toStdString() << " ";
	fout << ui->groundKd->toPlainText().toStdString() << " ";
	fout << ui->servoKs->toPlainText().toStdString() << " ";
	fout << ui->servoKd->toPlainText().toStdString() << " " << std::endl;;

	for (int i = 0; i < values.size(); i++)
		fout << values.at(i)->toPlainText().toStdString() << " ";

	fout.close();
}

void HuboTrackingManage::load(char *filename)
{
	std::ifstream fin;
	fin.open(filename);
	double tmp;
	fin >> tmp;
	ui->groundKs->setText(QString::number(tmp));
	fin >> tmp;
	ui->groundKd->setText(QString::number(tmp));
	fin >> tmp;
	ui->servoKs->setText( QString::number(tmp));
	fin >> tmp;
	ui->servoKd->setText( QString::number(tmp));

	for (int i = 0; i < values.size(); i++)
	{
		char value[32];
		fin >> tmp;
		sprintf(value, "%.3lf", tmp);
		values.at(i)->setText(QString(value));
	}

	fin.close();
}

void HuboTrackingManage::on_simulBtn_clicked()
{
	on_groundSetBtn_clicked();
	on_servoSetBtn_clicked();
	viewer->setCmaMotion(0, 1);
}

void HuboTrackingManage::on_runCmaWithThis_clicked()
{
	//TODO:

}

void HuboTrackingManage::on_runCmaWithRes_clicked()
{
    viewer->cmaRun(ui->cmaRunIterEdit->toPlainText().toInt(), 1);
    for (int i = 0; i < viewer->cma.solution.size(); i++)
    {
        double value = viewer->cma.solution.at(i);
        sliders.at(i)->setValue(value *1000. /M_PI + 500.);
    }
    viewer->setCmaMotion();
}

void HuboTrackingManage::on_runCmaBtn_clicked()
{
    viewer->cmaRun(ui->cmaRunIterEdit->toPlainText().toInt(), 0);
	for (int i = 0; i < viewer->cma.solution.size(); i++)
	{
		double value = viewer->cma.solution.at(i);
		sliders.at(i)->setValue(value *1000. /M_PI + 500.);
	}
	viewer->setCmaMotion();
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
    if(textMoveFlag)
        return;
	sliderMoveFlag = 1;
	for (int i = 0; i < sliders.size(); i++)
	{
		char value[32];
		viewer->manualSol.at(i) = ((double)sliders.at(i)->value() - 500)*M_PI / 1000.;
		sprintf(value, "%.3lf", viewer->manualSol.at(i));
		values.at(i)->setText(value);
	}
	sliderMoveFlag = 0;
	
}

void HuboTrackingManage::text_valueChanged()
{
	if (sliderMoveFlag)
		return;
    textMoveFlag =1;
    for (int i=0; i<values.size(); i++)
    {
        QString str = values.at(i)->toPlainText();
        if(str.size() < 5 || (str.at(0) == '-' && str.size() < 6) )
            return;
    }
    for (int i = 0; i < values.size(); i++)
    {
        double value = values.at(i)->toPlainText().toDouble();
        sliders.at(i)->setValue( value*1000. / M_PI +500. );
    }
    textMoveFlag = 0;

}

void HuboTrackingManage::on_saveBtn_clicked()
{
	save("../CmaData/manualSol.txt");
}

void HuboTrackingManage::on_loadBtn_clicked()
{
	load("../CmaData/manualSol.txt");
}
