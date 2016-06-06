#include "pcappsexperiment.h"
#include "ui_pcappsexperiment.h"

PCAppsExperiment::PCAppsExperiment(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCAppsExperiment)
{
    ui->setupUi(this);

    experimenter = new Experimenter;

    // add opengl widget
    glwidget = new GlWidget();
    ui->verticalLayout->addWidget(glwidget);
    // set graphics scene
    colorScene = new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT);
    depthScene = new QGraphicsScene(0,0,IMAGE_WIDTH,IMAGE_HEIGHT);
    ui->graphicsView_color->setScene(colorScene);
    ui->graphicsView_depth->setScene(depthScene);

    // allocate memory for opengl vertices
    gvm::InitVertices();
    gvm::AddCartesianAxes();
    gvm::ShowAddedVertices();

    // set default UI
    ui->radioButton_view_color->setChecked(true);
    ui->checkBox_normal->setChecked(true);
    g_frameIdx=0;
}

PCAppsExperiment::~PCAppsExperiment()
{
    delete ui;
}

void PCAppsExperiment::DisplayImage(QImage colorImg, QImage depthImg)
{
    QImage depthGray;
    ImageConverter::ConvertToGrayImage(depthImg, depthGray);

    colorScene->addPixmap(QPixmap::fromImage(colorImg));
    depthScene->addPixmap(QPixmap::fromImage(depthGray));
}

int PCAppsExperiment::GetViewOptions()
{
    int viewOption = ViewOpt::ViewNone;
    if(ui->radioButton_view_color->isChecked())
        viewOption |= ViewOpt::Color;
    else if(ui->radioButton_view_our_curvature->isChecked())
        viewOption |= ViewOpt::CURVATURE;
    else if(ui->radioButton_view_fpfh->isChecked())
        viewOption |= ViewOpt::FPFH;
    else if(ui->radioButton_view_shot->isChecked())
        viewOption |= ViewOpt::SHOT;
    if(ui->checkBox_normal->isChecked())
        viewOption |= ViewOpt::Normal;

    return viewOption;
}

void PCAppsExperiment::on_pushButton_virtual_depth_clicked()
{
    qDebug() << "==============================";
    qDebug() << "Virtual Frame:" << ++g_frameIdx;

    VirtualRgbdSensor sensor;
    const QString shapefile = QString(PCApps_PATH) + "/IO/VirtualConfig/shapes.txt";
    const QString camerafile = QString(PCApps_PATH) + "/IO/VirtualConfig/camera.txt";
    const QString noisefile = QString(PCApps_PATH) + "/IO/VirtualConfig/noise.txt";
    sensor.MakeVirtualDepth(shapefile, camerafile, noisefile);
    sensor.GrabFrame(colorImg, depthImg);
    DisplayImage(colorImg, depthImg);

    // point cloud work
    experimenter->Work(colorImg, depthImg, annots, &sharedData);

    // show point cloud on the screen
    UpdateView();
}

void PCAppsExperiment::on_pushButton_resetView_clicked()
{
    glwidget->ResetView();
}

void PCAppsExperiment::on_radioButton_view_color_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void PCAppsExperiment::on_radioButton_view_our_curvature_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void PCAppsExperiment::on_radioButton_view_fpfh_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void PCAppsExperiment::on_radioButton_view_shot_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void PCAppsExperiment::on_checkBox_normal_toggled(bool checked)
{
    if(checked)
        UpdateView();
}

void PCAppsExperiment::UpdateView()
{
    if(sharedData.NullData())
        return;
    int viewOption = GetViewOptions();
    DrawUtils::DrawPointCloud(viewOption, &sharedData);
    DisplayImage(colorImg, depthImg);
    gvm::AddCartesianAxes();
    gvm::ShowAddedVertices();
}
