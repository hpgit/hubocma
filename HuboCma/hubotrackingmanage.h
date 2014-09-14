#ifndef HUBOTRACKINGMANAGE_H
#define HUBOTRACKINGMANAGE_H

#include <QDialog>
#include "hubotrackingviewer.h"
#include <QtWidgets>

namespace Ui {
class HuboTrackingManage;
}

class HuboTrackingManage : public QDialog
{
    Q_OBJECT

public:
    explicit HuboTrackingManage(QWidget *parent = 0);
    ~HuboTrackingManage();

	void init(HuboTrackingViewer *_viewer);

	HuboTrackingViewer *viewer;
	std::vector<QTextEdit*>values;
	std::vector<QSlider*> sliders;

private slots:
    void text_valueChanged();
	void slider_valueChanged();

	void on_groundSetBtn_clicked();

	void on_servoSetBtn_clicked();

    void on_simulBtn_clicked();

    void on_runCmaWithThis_clicked();

    void on_runCmaWithRes_clicked();

private:
    Ui::HuboTrackingManage *ui;
};

#endif // HUBOTRACKINGMANAGE_H
