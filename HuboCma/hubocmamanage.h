#ifndef HUBOCMAMANAGE_H
#define HUBOCMAMANAGE_H

#include <QDialog>
#include <QtWidgets>
#include "hubotrackingviewer.h"

namespace Ui {
class HuboCmaManage;
}

class HuboCmaManage : public QDialog
{
    Q_OBJECT

public:
    explicit HuboCmaManage(QWidget *parent = 0);
    ~HuboCmaManage();

	void initManager(HuboTrackingViewer *_viewer);

	HuboTrackingViewer *viewer;

	void physSetting();

private slots:
	void on_runCmaBtn_clicked();
	void on_runCmaWithResBtn_clicked();
	void on_loadBtn_clicked();
	void on_saveBtn_clicked();
	void on_pauseBtn_clicked();
	void on_resumeBtn_clicked();
	void on_stopBtn_clicked();
	void on_simulateBtn_clicked();
	

private:
    Ui::HuboCmaManage *ui;
};

#endif // HUBOCMAMANAGE_H
