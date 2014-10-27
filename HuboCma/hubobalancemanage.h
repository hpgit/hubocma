#ifndef HUBOBALANCEMANAGE_H
#define HUBOBALANCEMANAGE_H

#include <QDialog>
#include "hubobalanceviewer.h"

namespace Ui {
class HuboBalanceManage;
}

class HuboBalanceManage : public QDialog
{
	Q_OBJECT

public:
	explicit HuboBalanceManage(QWidget *parent = 0);
	~HuboBalanceManage();
	HuboBalanceViewer *viewer;

private slots:
	void on_goBtn_clicked();

private:
	Ui::HuboBalanceManage *ui;
};

#endif // HUBOBALANCEMANAGE_H
