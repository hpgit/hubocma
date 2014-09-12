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

void HuboTrackingManage::on_simulBtn_clicked()
{

}

void HuboTrackingManage::on_runCmaWithThis_clicked()
{

}

void HuboTrackingManage::on_runCmaWithRes_clicked()
{

}
