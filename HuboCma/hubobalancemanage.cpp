#include "hubobalancemanage.h"
#include "ui_hubobalancemanage.h"

HuboBalanceManage::HuboBalanceManage(QWidget *parent) :
	QDialog(parent),
	ui(new Ui::HuboBalanceManage)
{
	ui->setupUi(this);
	//ui->textEdit  ->setText(QString("100"));
	//ui->textEdit_2->setText(QString("100"));
	//ui->textEdit_3->setText(QString("200"));
	//ui->textEdit_4->setText(QString("50"));
	//ui->textEdit_5->setText(QString("50"));
	//ui->textEdit_6->setText(QString("30"));
	ui->textEdit  ->setText(QString("1"));
	ui->textEdit_2->setText(QString("1"));
	ui->textEdit_3->setText(QString("1"));
	ui->textEdit_4->setText(QString("1"));
	ui->textEdit_5->setText(QString("1"));
	ui->textEdit_6->setText(QString("30"));
}

HuboBalanceManage::~HuboBalanceManage()
{
	delete ui;
}
void HuboBalanceManage::on_goBtn_clicked()
{
	int frameRate;
	double kl, kh, wTra, wTraAnkle, wTraUpper ;
	kl = ui->textEdit->toPlainText().toDouble();
	kh = ui->textEdit_2->toPlainText().toDouble();
	wTra = ui->textEdit_3->toPlainText().toDouble();
	wTraAnkle = ui->textEdit_4->toPlainText().toDouble();
	wTraUpper = ui->textEdit_5->toPlainText().toDouble();
	frameRate = ui->textEdit_6->toPlainText().toInt();
	//viewer->setCmaMotion(kl, kh, wTra, wTraAnkle, wTraUpper, frameRate);
	viewer->setBalanceMotion(kl, kh, wTra, wTraAnkle, wTraUpper, frameRate);
}
