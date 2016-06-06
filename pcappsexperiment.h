#ifndef EXPERIMENTWINDOW_H
#define EXPERIMENTWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QTimer>
#include <QMouseEvent>
#include "Share/project_common.h"
#include "Share/shared_data.h"
#include "ShareExpm/expm_common.h"
#include "IO/glwidget.h"
#include "IO/glvertexmanager.h"
#include "IO/rgbdfilerw.h"
#include "IO/imageconverter.h"
#include "IO/drawutils.h"
#include "IO/VirtualSensor/virtualrgbdsensor.h"

#include "Experiment/experimenter.h"

namespace Ui {
class PCAppsExperiment;
}

class PCAppsExperiment : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCAppsExperiment(QWidget *parent = 0);
    ~PCAppsExperiment();

private slots:
    void on_pushButton_virtual_depth_clicked();
    void on_pushButton_resetView_clicked();
    void on_radioButton_view_color_toggled(bool checked);
    void on_radioButton_view_our_curvature_toggled(bool checked);
    void on_radioButton_view_fpfh_toggled(bool checked);
    void on_radioButton_view_shot_toggled(bool checked);
    void on_checkBox_normal_toggled(bool checked);

private:
    void DisplayImage(QImage colorImg, QImage depthImg);
    int GetViewOptions();
    void UpdateView();

    Ui::PCAppsExperiment *ui;
    GlWidget* glwidget;
    QGraphicsScene* colorScene;
    QGraphicsScene* depthScene;
    QImage colorImg;
    QImage depthImg;
    vecAnnot annots;
    SharedData sharedData;

    Experimenter* experimenter;
};

#endif // EXPERIMENTWINDOW_H
