#pragma once

#include <QWidget>
#include <QVector3D>
#include <QQuaternion>

#include "types.h"

namespace Ui {
class vehicle_state_table;
}

class VehicleStateTable : public QWidget
{
    Q_OBJECT

public:
    explicit VehicleStateTable(QWidget *parent = nullptr);
    ~VehicleStateTable();

public slots:
    void set_vehicle_state(const QVector3D &pose, const EulerKrylovAngles& angles);

private:
    Ui::vehicle_state_table *ui;
};
