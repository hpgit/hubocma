#include "hubomaincontroller.h"
#include "ui_hubomaincontroller.h"

HuboMainController::HuboMainController(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::HuboMainController)
{
    ui->setupUi(this);
}

HuboMainController::~HuboMainController()
{
    delete ui;
}
