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

    clouds_vis_ = new CloudsVisualizer(vehicle_model_path);
    vehicle_state_table_ = new VehicleStateTable();
    attitude_ind_ = new AttitudeIndicator();
    compass_ = new Compass();
    test_widget_ = new TestWidget();

    setup_layout_(vehicle_model_path);

    test_widget_->setVisible(show_test_widget_);

    connect(test_widget_, &TestWidget::change_vehicle_pose, this, &MainWindow::set_vehicle_pose_);
}

void MainWindow::setup_layout_(const QString& vehicle_model_path)
{
    ui->verticalLayout->addWidget(clouds_vis_);

    auto h_layout = new QHBoxLayout();
    h_layout->addWidget(vehicle_state_table_);
    h_layout->addWidget(attitude_ind_);
    h_layout->addWidget(compass_);
    h_layout->addWidget(test_widget_);
    h_layout->addSpacerItem(new QSpacerItem(1,1, QSizePolicy::Expanding, QSizePolicy::Fixed));

    ui->verticalLayout->addLayout(h_layout);

    clouds_vis_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    vehicle_state_table_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    attitude_ind_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    compass_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    test_widget_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
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
    auto angles = convert_to_angles(quaternion);
    set_vehicle_pose_(pose, quaternion, angles);
}

void MainWindow::keyPressEvent(QKeyEvent *pe)
{
    switch (pe->key())
    {
        case Qt::Key_F1:
            show_hide_test_widget_();
            pe->accept();
            update();
        break;
        default:
            clouds_vis_->keyPressEvent(pe);
        break;
    }
}

void MainWindow::show_hide_test_widget_()
{
    show_test_widget_ = !show_test_widget_;
    test_widget_->setVisible(show_test_widget_);
}

void MainWindow::set_vehicle_pose_(const QVector3D &pose, const QQuaternion &quaternion, const EulerKrylovAngles &angles)
{
    clouds_vis_->draw_vehicle(pose, quaternion);

    vehicle_state_table_->set_vehicle_state(pose, angles);
    attitude_ind_->set_data(angles.roll, angles.pitch);
    compass_->set_course(angles.cource);
}
