#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QHBoxLayout>
#include <QSizePolicy>

#include "utils.h"

MainWindow::MainWindow(const QString& vehicle_model_path, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setup_layout(vehicle_model_path);
}

void MainWindow::setup_layout(const QString& vehicle_model_path)
{
    clouds_vis_ = new CloudsVisualizer(vehicle_model_path);
    vehicle_state_table_ = new VehicleStateTable();
    attitude_ind_ = new AttitudeIndicator();
    compass_ = new Compass();

    ui->verticalLayout->addWidget(clouds_vis_);

    auto h_layout = new QHBoxLayout();
    h_layout->addWidget(vehicle_state_table_);
    h_layout->addWidget(attitude_ind_);
    h_layout->addWidget(compass_);
    h_layout->addSpacerItem(new QSpacerItem(1,1, QSizePolicy::Expanding, QSizePolicy::Fixed));

    ui->verticalLayout->addLayout(h_layout);

    clouds_vis_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    vehicle_state_table_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    attitude_ind_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    compass_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
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

    auto angles = convert_to_angles(quaternion);

    vehicle_state_table_->set_vehicle_state(pose, angles);
    attitude_ind_->set_data(angles.roll, angles.pitch);
    compass_->set_course(angles.cource);
}
