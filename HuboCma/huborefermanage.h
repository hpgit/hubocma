#ifndef HUBOREFERMANAGE_H
#define HUBOREFERMANAGE_H

#include <QDialog>
#include "huboreferviewer.h"

namespace Ui {
class HuboReferManage;
}

class HuboReferManage : public QDialog
{
	Q_OBJECT

public:
	explicit HuboReferManage(QWidget *parent = 0);
	~HuboReferManage();

	HuboReferViewer *viewer;
private slots:
	void on_saveBtn_clicked();

	void on_cutFrontBtn_clicked();

	void on_cutBackBtn_clicked();

private:
	Ui::HuboReferManage *ui;
};

#endif // HUBOREFERMANAGE_H
