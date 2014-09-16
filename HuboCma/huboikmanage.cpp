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
}

void HuboIkManage::on_rFootBtn_clicked()
{
	viewer->rFoot();
}