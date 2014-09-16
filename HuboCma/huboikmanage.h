#ifndef HUBOIKMANAGE_H
#define HUBOIKMANAGE_H

#include <QDialog>
#include <QtWidgets>
#include "huboikviewer.h"

namespace Ui {class HuboIkManage;};

class HuboIkManage : public QDialog
{
	Q_OBJECT

public:
	HuboIkManage(QWidget *parent = 0);
	~HuboIkManage();

	HuboIkViewer *viewer;

	void init(HuboIkViewer *viewer);

private slots:
	void on_rFootBtn_clicked();

private:
	Ui::HuboIkManage *ui;
};

#endif // HUBOIKMANAGE_H
