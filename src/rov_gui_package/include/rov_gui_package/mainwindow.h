#pragma once

#include <QMainWindow>
#include <QVector>
#include <QVector3D>
#include <QQuaternion>

#include "cloudsvisualizer.h"
#include "vehiclestatetable.h"
#include "compass.h"
#include "attitudeindicator.h"
#include "testwidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(const QString& vehicle_model_path = "", QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void draw_map(const QVector<QVector3D>& points);
    void draw_sonar_data(const QVector<QVector3D>& points);
    void draw_vehicle(const QVector3D& pose, const QQuaternion& quaternion = QQuaternion());

protected:
    void keyPressEvent(QKeyEvent* pe) override;

private:
    Ui::MainWindow *ui;

    CloudsVisualizer* clouds_vis_;
    VehicleStateTable* vehicle_state_table_;

    AttitudeIndicator* attitude_ind_;
    Compass* compass_;
    TestWidget* test_widget_;
    bool show_test_widget_ = false;

    void setup_layout_(const QString& vehicle_model_path);
    void show_hide_test_widget_();
    void set_vehicle_pose_(const QVector3D& pose, const QQuaternion& quaternion, const EulerKrylovAngles& angles);
};
