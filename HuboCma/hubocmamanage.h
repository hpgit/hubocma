#ifndef HUBOCMAMANAGE_H
#define HUBOCMAMANAGE_H

#include <QDialog>

namespace Ui {
class HuboCmaManage;
}

class HuboCmaManage : public QDialog
{
    Q_OBJECT

public:
    explicit HuboCmaManage(QWidget *parent = 0);
    ~HuboCmaManage();

private:
    Ui::HuboCmaManage *ui;
};

#endif // HUBOCMAMANAGE_H
