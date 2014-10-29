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
	void on_solveBtn_clicked();

	void on_backupBtn_clicked();

	void on_initBtn_clicked();

	void on_initAllBtn_clicked();

	void on_propagateBtn_clicked();

	void on_saveBtn_clicked();

	void on_applyBtn_clicked();

	void on_solveAllBtn_clicked();


private:
	Ui::HuboIkManage *ui;
};

#endif // HUBOIKMANAGE_H
