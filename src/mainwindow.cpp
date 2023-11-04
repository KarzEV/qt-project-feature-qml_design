#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(const QString& vehicle_model_path, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    clouds_vis_ = new CloudsVisualizer(vehicle_model_path);
    ui->verticalLayout->addWidget(clouds_vis_);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::draw_map(const QVector<QVector3D> &points)
{
    clouds_vis_->draw_map(points);
}

void MainWindow::draw_sonar_data(const QVector<QVector3D> &points)
{
    clouds_vis_->draw_sonar_data(points);
}

void MainWindow::draw_vehicle(const QVector3D &pose, const QQuaternion &quaternion)
{
    clouds_vis_->draw_vehicle(pose, quaternion);
}
