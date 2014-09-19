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
