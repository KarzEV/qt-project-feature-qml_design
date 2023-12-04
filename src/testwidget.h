#ifndef TESTWIDGET_H
#define TESTWIDGET_H

#include <QWidget>
#include <QVector3D>
#include <QQuaternion>

#include "types.h"

namespace Ui {
class TestWidget;
}

class TestWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TestWidget(QWidget *parent = nullptr);
    ~TestWidget();

signals:
    void change_vehicle_pose(const QVector3D& pose, const QQuaternion& quaternion, const EulerKrylovAngles& angles);

private slots:
    void on_X_Slider_valueChanged(int value);
    void on_Y_Slider_valueChanged(int value);
    void on_Z_Slider_valueChanged(int value);

    void on_Cource_Slider_valueChanged(int value);
    void on_Roll_Slider_valueChanged(int value);
    void on_Pitch_Slider_valueChanged(int value);

private:
    Ui::TestWidget *ui;

    QVector3D pose_;
    EulerKrylovAngles angles_;

    void setup_layout_();
    void send_new_vehicle_pose_();
};

#endif // TESTWIDGET_H
