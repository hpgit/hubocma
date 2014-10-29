#include "huboreferviewer.h"
#include "huborefermanage.h"

HuboReferViewer::HuboReferViewer(QWidget *parent)
{
	//TODO:
	//Making manager
	//1. cutting original motion
	//2. annotating contact info

	HuboReferManage *manager = new HuboReferManage;
	manager->viewer = this;
	manager->show();

}

